#include "aos/events/logging/log_backend.h"

#include <dirent.h>

#include <filesystem>

#include "absl/strings/str_cat.h"
#include "aos/util/file.h"
#include "glog/logging.h"

DEFINE_bool(direct, false,
            "If true, write using O_DIRECT and write 512 byte aligned blocks "
            "whenever possible.");
DEFINE_bool(
    sync, false,
    "If true, sync data to disk as we go so we don't get too far ahead.  Also "
    "fadvise that we are done with the memory once it hits disk.");

DEFINE_uint32(queue_reserve, 32, "Pre-reserved size of write queue.");

namespace aos::logger {
namespace {
constexpr const char *kTempExtension = ".tmp";

// Assuming that kSector is power of 2, it aligns address to the left size.
inline size_t AlignToLeft(size_t value) {
  return value & (~(FileHandler::kSector - 1));
}

inline bool IsAligned(size_t value) {
  return value % FileHandler::kSector == 0;
}

inline bool IsAlignedStart(const absl::Span<const uint8_t> span) {
  return (reinterpret_cast<size_t>(span.data()) % FileHandler::kSector) == 0;
}

inline bool IsAlignedLength(const absl::Span<const uint8_t> span) {
  return (span.size() % FileHandler::kSector) == 0;
}

}  // namespace

logger::QueueAligner::QueueAligner() {
  aligned_queue_.reserve(FLAGS_queue_reserve);
}

void logger::QueueAligner::FillAlignedQueue(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  aligned_queue_.clear();

  for (const auto &span : queue) {
    // Generally, every span might have 3 optional parts (i.e. 2^3 cases):
    // 1. unaligned prefix -  from start till first aligned block.
    // 2. aligned main - block with aligned start and size
    // 3. unaligned suffix - block with aligned start, and size less than one
    // sector. If size of the span is less than 1 sector, let's call it prefix.

    auto *data = span.data();
    size_t size = span.size();
    const auto start = reinterpret_cast<size_t>(data);
    VLOG(2) << "Consider span starting at " << std::hex << start
            << " with size " << size;

    CHECK_GT(size, 0u) << ": Nobody should be sending empty messages.";

    const auto next_aligned =
        IsAligned(start) ? start : AlignToLeft(start) + FileHandler::kSector;
    const auto prefix_size = next_aligned - start;
    VLOG(2) << "Calculated prefix size " << std::hex << prefix_size;

    if (prefix_size >= size) {
      // size of prefix >= size of span - alignment is not possible, accept the
      // whole span
      VLOG(2) << "Only prefix found";
      CHECK_GT(size, 0u);
      aligned_queue_.emplace_back(data, size, false);
      continue;
    }
    CHECK_LT(prefix_size, FileHandler::kSector)
        << ": Wrong calculation of 'next' aligned position";
    if (prefix_size > 0) {
      // Cut the prefix and move to the main part.
      VLOG(2) << "Cutting prefix at " << std::hex << start << " of size "
              << prefix_size;
      aligned_queue_.emplace_back(data, prefix_size, false);
      data += prefix_size;
      size -= prefix_size;
      CHECK(data <= span.data() + span.size()) << " :Boundaries after prefix";
    }

    if (IsAligned(size)) {
      // the rest is aligned.
      VLOG(2) << "Returning aligned main part";
      CHECK_GT(size, 0u);
      aligned_queue_.emplace_back(data, size, true);
      continue;
    }

    const auto aligned_size = AlignToLeft(size);
    CHECK(aligned_size < size) << ": Wrong calculation of 'main' size";
    if (aligned_size > 0) {
      VLOG(2) << "Cutting main part starting " << std::hex
              << reinterpret_cast<size_t>(data) << " of size " << aligned_size;
      aligned_queue_.emplace_back(data, aligned_size, true);

      data += aligned_size;
      size -= aligned_size;
      CHECK(data <= span.data() + span.size()) << " :Boundaries after main";
    }

    VLOG(2) << "Cutting suffix part starting " << std::hex
            << reinterpret_cast<size_t>(data) << " of size " << size;
    CHECK_GT(size, 0u);
    aligned_queue_.emplace_back(data, size, false);
  }
}

FileHandler::FileHandler(std::string filename)
    : filename_(std::move(filename)), supports_odirect_(FLAGS_direct) {}

FileHandler::~FileHandler() { Close(); }

WriteCode FileHandler::OpenForWrite() {
  iovec_.reserve(10);
  if (!aos::util::MkdirPIfSpace(filename_, 0777)) {
    return WriteCode::kOutOfSpace;
  } else {
    fd_ = open(filename_.c_str(), O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774);
    if (fd_ == -1 && errno == ENOSPC) {
      return WriteCode::kOutOfSpace;
    } else {
      PCHECK(fd_ != -1) << ": Failed to open " << filename_ << " for writing";
      VLOG(1) << "Opened " << filename_ << " for writing";
    }

    flags_ = fcntl(fd_, F_GETFL, 0);
    PCHECK(flags_ >= 0) << ": Failed to get flags for " << filename_;

    EnableDirect();

    CHECK(std::filesystem::exists(filename_));

    return WriteCode::kOk;
  }
}

void FileHandler::EnableDirect() {
  if (supports_odirect_ && !ODirectEnabled()) {
    const int new_flags = flags_ | O_DIRECT;
    // Track if we failed to set O_DIRECT.  Note: Austin hasn't seen this call
    // fail.  The write call tends to fail instead.
    if (fcntl(fd_, F_SETFL, new_flags) == -1) {
      PLOG(WARNING) << "Failed to set O_DIRECT on " << filename_;
      supports_odirect_ = false;
    } else {
      VLOG(1) << "Enabled O_DIRECT on " << filename_;
      flags_ = new_flags;
    }
  }
}

void FileHandler::DisableDirect() {
  if (supports_odirect_ && ODirectEnabled()) {
    flags_ = flags_ & (~O_DIRECT);
    PCHECK(fcntl(fd_, F_SETFL, flags_) != -1) << ": Failed to disable O_DIRECT";
    VLOG(1) << "Disabled O_DIRECT on " << filename_;
  }
}

WriteResult FileHandler::Write(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  iovec_.clear();
  CHECK_LE(queue.size(), static_cast<size_t>(IOV_MAX));

  queue_aligner_.FillAlignedQueue(queue);
  CHECK_LE(queue_aligner_.aligned_queue().size(), static_cast<size_t>(IOV_MAX));

  // Ok, we now need to figure out if we were aligned, and if we were, how much
  // of the data we are being asked to write is aligned.
  //
  // When writing with O_DIRECT, the kernel only will accept writes where the
  // offset into the file is a multiple of kSector, the data is aligned to
  // kSector in memory, and the length being written is a multiple of kSector.
  // Some of the callers use an aligned ResizeableBuffer to generate 512 byte
  // aligned buffers for this code to find and use.
  bool was_aligned = IsAligned(total_write_bytes_);
  VLOG(1) << "Started " << (was_aligned ? "aligned" : "unaligned")
          << " at offset " << total_write_bytes_ << " on " << filename();

  // Walk through aligned queue and batch writes based on aligned flag
  for (const auto &item : queue_aligner_.aligned_queue()) {
    if (was_aligned != item.aligned) {
      // Switching aligned context. Let's flush current batch.
      if (!iovec_.empty()) {
        // Flush current queue if we need.
        const auto code = WriteV(iovec_, was_aligned);
        if (code == WriteCode::kOutOfSpace) {
          // We cannot say anything about what number of messages was written
          // for sure.
          return {
              .code = code,
              .messages_written = queue.size(),
          };
        }
        iovec_.clear();
      }
      // Write queue is flushed. WriteV updates the total_write_bytes_.
      was_aligned = IsAligned(total_write_bytes_) && item.aligned;
    }
    iovec_.push_back(
        {.iov_base = const_cast<uint8_t *>(item.data), .iov_len = item.size});
  }

  WriteCode result_code = WriteCode::kOk;
  if (!iovec_.empty()) {
    // Flush current queue if we need.
    result_code = WriteV(iovec_, was_aligned);
  }
  return {
      .code = result_code,
      .messages_written = queue.size(),
  };
}

WriteCode FileHandler::WriteV(const std::vector<struct iovec> &iovec,
                              bool aligned) {
  // Configure the file descriptor to match the mode we should be in.  This is
  // safe to over-call since it only does the syscall if needed.
  if (aligned) {
    EnableDirect();
  } else {
    DisableDirect();
  }

  VLOG(2) << "Flushing queue of " << iovec.size() << " elements, "
          << (aligned ? "aligned" : "unaligned");

  CHECK_GT(iovec.size(), 0u);
  const auto start = aos::monotonic_clock::now();

  // Validation of alignment assumptions.
  if (aligned) {
    CHECK(IsAligned(total_write_bytes_))
        << ": Failed after writing " << total_write_bytes_
        << " to the file, attempting aligned write with unaligned start.";

    for (const auto &iovec_item : iovec) {
      absl::Span<const uint8_t> data(
          reinterpret_cast<const uint8_t *>(iovec_item.iov_base),
          iovec_item.iov_len);
      VLOG(2) << "  iov_base " << static_cast<void *>(iovec_item.iov_base)
              << ", iov_len " << iovec_item.iov_len;
      CHECK(IsAlignedStart(data) && IsAlignedLength(data));
    }
  }

  // Calculation of expected written size.
  size_t counted_size = 0;
  for (const auto &iovec_item : iovec) {
    CHECK_GT(iovec_item.iov_len, 0u);
    counted_size += iovec_item.iov_len;
  }

  VLOG(2) << "Going to write " << counted_size;
  CHECK_GT(counted_size, 0u);

  const ssize_t written = writev(fd_, iovec.data(), iovec.size());
  VLOG(2) << "Wrote " << written << ", for iovec size " << iovec.size();

  const auto end = aos::monotonic_clock::now();
  if (written == -1 && errno == ENOSPC) {
    return WriteCode::kOutOfSpace;
  }
  PCHECK(written >= 0) << ": write failed, got " << written;
  if (written < static_cast<ssize_t>(counted_size)) {
    // Sometimes this happens instead of ENOSPC. On a real filesystem, this
    // never seems to happen in any other case. If we ever want to log to a
    // socket, this will happen more often. However, until we get there, we'll
    // just assume it means we ran out of space.
    return WriteCode::kOutOfSpace;
  }

  if (FLAGS_sync) {
    // Flush asynchronously and force the data out of the cache.
    sync_file_range(fd_, total_write_bytes_, written, SYNC_FILE_RANGE_WRITE);
    if (last_synced_bytes_ != 0) {
      // Per Linus' recommendation online on how to do fast file IO, do a
      // blocking flush of the previous write chunk, and then tell the kernel to
      // drop the pages from the cache.  This makes sure we can't get too far
      // ahead.
      sync_file_range(fd_, last_synced_bytes_,
                      total_write_bytes_ - last_synced_bytes_,
                      SYNC_FILE_RANGE_WAIT_BEFORE | SYNC_FILE_RANGE_WRITE |
                          SYNC_FILE_RANGE_WAIT_AFTER);
      posix_fadvise(fd_, last_synced_bytes_,
                    total_write_bytes_ - last_synced_bytes_,
                    POSIX_FADV_DONTNEED);
    }
    last_synced_bytes_ = total_write_bytes_;
  }

  total_write_bytes_ += written;
  if (aligned) {
    written_aligned_ += written;
  }
  WriteStatistics()->UpdateStats(end - start, written, iovec.size());
  return WriteCode::kOk;
}

WriteCode FileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }
  bool ran_out_of_space = false;
  if (close(fd_) == -1) {
    if (errno == ENOSPC) {
      ran_out_of_space = true;
    } else {
      PLOG(ERROR) << "Closing log file failed";
    }
  }
  fd_ = -1;
  VLOG(1) << "Closed " << filename_;
  return ran_out_of_space ? WriteCode::kOutOfSpace : WriteCode::kOk;
}

FileBackend::FileBackend(std::string_view base_name)
    : base_name_(base_name), separator_(base_name_.back() == '/' ? "" : "_") {}

std::unique_ptr<LogSink> FileBackend::RequestFile(std::string_view id) {
  const std::string filename = absl::StrCat(base_name_, separator_, id);
  return std::make_unique<FileHandler>(filename);
}

RenamableFileBackend::RenamableFileBackend(std::string_view base_name)
    : base_name_(base_name), separator_(base_name_.back() == '/' ? "" : "_") {}

std::unique_ptr<LogSink> RenamableFileBackend::RequestFile(
    std::string_view id) {
  const std::string filename =
      absl::StrCat(base_name_, separator_, id, temp_suffix_);
  return std::make_unique<RenamableFileHandler>(this, filename);
}

void RenamableFileBackend::EnableTempFiles() {
  use_temp_files_ = true;
  temp_suffix_ = kTempExtension;
}

bool RenamableFileBackend::RenameLogBase(std::string_view new_base_name) {
  if (new_base_name == base_name_) {
    return true;
  }
  CHECK(old_base_name_.empty())
      << "Only one change of base_name is supported. Was: " << old_base_name_;

  std::string current_directory = base_name_;
  std::string new_directory(new_base_name);

  auto current_path_split = current_directory.rfind("/");
  CHECK(current_path_split != std::string::npos)
      << "Could not find / in the current directory path";
  auto new_path_split = new_directory.rfind("/");
  CHECK(new_path_split != std::string::npos)
      << "Could not find / in the new directory path";

  CHECK(new_base_name.substr(new_path_split) ==
        current_directory.substr(current_path_split))
      << "Rename of file base from " << current_directory << " to "
      << new_directory << " is not supported.";

  current_directory.resize(current_path_split);
  new_directory.resize(new_path_split);
  DIR *dir = opendir(current_directory.c_str());
  if (dir) {
    closedir(dir);
    const int result = rename(current_directory.c_str(), new_directory.c_str());
    if (result != 0) {
      PLOG(ERROR) << "Unable to rename " << current_directory << " to "
                  << new_directory;
      return false;
    }
  } else {
    // Handle if directory was already renamed.
    dir = opendir(new_directory.c_str());
    if (!dir) {
      LOG(ERROR) << "Old directory " << current_directory
                 << " missing and new directory " << new_directory
                 << " not present.";
      return false;
    }
    closedir(dir);
  }
  old_base_name_ = base_name_;
  base_name_ = std::string(new_base_name);
  separator_ = base_name_.back() == '/' ? "" : "_";
  return true;
}

WriteCode RenamableFileBackend::RenameFileAfterClose(
    std::string_view filename) {
  // Fast check that we can skip rename.
  if (!use_temp_files_ && old_base_name_.empty()) {
    return WriteCode::kOk;
  }

  std::string current_filename(filename);

  // When changing the base name, we rename the log folder while there active
  // buffer writers. Therefore, the name of that active buffer may still refer
  // to the old file location rather than the new one.
  if (!old_base_name_.empty()) {
    auto offset = current_filename.find(old_base_name_);
    if (offset != std::string::npos) {
      current_filename.replace(offset, old_base_name_.length(), base_name_);
    }
  }

  std::string final_filename = current_filename;
  if (use_temp_files_) {
    CHECK(current_filename.size() > temp_suffix_.size());
    final_filename = current_filename.substr(
        0, current_filename.size() - temp_suffix_.size());
  }

  int result = rename(current_filename.c_str(), final_filename.c_str());

  bool ran_out_of_space = false;
  if (result != 0) {
    if (errno == ENOSPC) {
      ran_out_of_space = true;
    } else {
      PLOG(FATAL) << "Renaming " << current_filename << " to " << final_filename
                  << " failed";
    }
  } else {
    VLOG(1) << "Renamed " << current_filename << " -> " << final_filename;
  }
  return ran_out_of_space ? WriteCode::kOutOfSpace : WriteCode::kOk;
}

WriteCode RenamableFileBackend::RenamableFileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }
  if (FileHandler::Close() == WriteCode::kOutOfSpace) {
    return WriteCode::kOutOfSpace;
  }
  if (owner_->RenameFileAfterClose(filename()) == WriteCode::kOutOfSpace) {
    return WriteCode::kOutOfSpace;
  }
  return WriteCode::kOk;
}

}  // namespace aos::logger
