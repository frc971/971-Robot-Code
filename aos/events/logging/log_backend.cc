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

namespace aos::logger {
namespace {
constexpr const char *kTempExtension = ".tmp";
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
    PCHECK(flags_ >= 0) << ": Failed to get flags for " << this->filename();

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
      PLOG(WARNING) << "Failed to set O_DIRECT on " << filename();
      supports_odirect_ = false;
    } else {
      VLOG(1) << "Enabled O_DIRECT on " << filename();
      flags_ = new_flags;
    }
  }
}

void FileHandler::DisableDirect() {
  if (supports_odirect_ && ODirectEnabled()) {
    flags_ = flags_ & (~O_DIRECT);
    PCHECK(fcntl(fd_, F_SETFL, flags_) != -1) << ": Failed to disable O_DIRECT";
    VLOG(1) << "Disabled O_DIRECT on " << filename();
  }
}

inline bool IsAlignedStart(const absl::Span<const uint8_t> span) {
  return (reinterpret_cast<size_t>(span.data()) % FileHandler::kSector) == 0;
}

inline bool IsAlignedLength(const absl::Span<const uint8_t> span) {
  return (span.size() % FileHandler::kSector) == 0;
}

inline bool HasAtleastOneSector(const absl::Span<const uint8_t> span) {
  return span.size() >= FileHandler::kSector;
}

WriteResult FileHandler::Write(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  iovec_.clear();
  CHECK_LE(queue.size(), static_cast<size_t>(IOV_MAX));
  iovec_.resize(queue.size());
  // Size of the data currently in iovec_.
  size_t counted_size = 0;

  // Ok, we now need to figure out if we were aligned, and if we were, how much
  // of the data we are being asked to write is aligned.
  //
  // When writing with O_DIRECT, the kernel only will accept writes where the
  // offset into the file is a multiple of kSector, the data is aligned to
  // kSector in memory, and the length being written is a multiple of kSector.
  // Some of the callers use an aligned ResizeableBuffer to generate 512 byte
  // aligned buffers for this code to find and use.
  bool was_aligned = (total_write_bytes_ % kSector) == 0;

  if (was_aligned) {
    VLOG(1) << "Started aligned at offset " << total_write_bytes_ << " on "
            << filename();
  } else {
    VLOG(1) << "Started unaligned at offset " << total_write_bytes_ << " on "
            << filename();
  }

  // Index we are filling in next.  Keeps resetting back to 0 as we write
  // intermediates.
  size_t write_index = 0;
  for (size_t i = 0; i < queue.size(); ++i) {
    iovec_[write_index].iov_base = const_cast<uint8_t *>(queue[i].data());
    iovec_[write_index].iov_len = queue[i].size();

    // Make sure the address is aligned, or give up.  This should be uncommon,
    // but is always possible.
    if (!IsAlignedStart(queue[i])) {
      // Flush if we were aligned and have data.
      if (was_aligned && write_index != 0) {
        VLOG(1) << "Was aligned, now is not, writing previous data";
        const auto code =
            WriteV(iovec_.data(), write_index, true, counted_size);
        if (code == WriteCode::kOutOfSpace) {
          return {
              .code = code,
              .messages_written = i,
          };
        }

        // Now, everything before here has been written.  Make an iovec out of
        // the rest and keep going.
        write_index = 0;
        counted_size = 0;

        iovec_[write_index].iov_base = const_cast<uint8_t *>(queue[i].data());
        iovec_[write_index].iov_len = queue[i].size();
      }
      was_aligned = false;
    } else {
      // We are now starting aligned again, and have data worth writing! Flush
      // what was there before.
      if (!was_aligned && HasAtleastOneSector(queue[i]) &&
          ((total_write_bytes_ + counted_size) % kSector) == 0 &&
          write_index != 0) {
        VLOG(1) << "Was not aligned, now is, writing previous data";

        const auto code =
            WriteV(iovec_.data(), write_index, false, counted_size);
        if (code == WriteCode::kOutOfSpace) {
          return {
              .code = code,
              .messages_written = i,
          };
        }

        // Now, everything before here has been written.  Make an iovec out of
        // the rest and keep going.
        write_index = 0;
        counted_size = 0;

        iovec_[write_index].iov_base = const_cast<uint8_t *>(queue[i].data());
        iovec_[write_index].iov_len = queue[i].size();
        was_aligned = true;
      }
    }

    // Now, see if the length is a multiple of kSector.  The goal is to figure
    // out if/how much memory we can write out with O_DIRECT so that only the
    // last little bit is done with non-direct IO to keep it fast.
    if (!IsAlignedLength(queue[i])) {
      VLOG(1) << "Unaligned length " << queue[i].size() << " on " << filename();
      // If we've got over a sector of data to write, write it out with
      // O_DIRECT and then continue writing the rest unaligned.
      if (was_aligned) {
        if (!HasAtleastOneSector(queue[i])) {
          if (write_index > 0) {
            const auto code =
                WriteV(iovec_.data(), write_index, true, counted_size);
            if (code == WriteCode::kOutOfSpace) {
              return {
                  .code = code,
                  .messages_written = i,
              };
            }

            // Now, everything before here has been written.  Make an iovec out
            // of the rest and keep going.
            write_index = 0;
            counted_size = 0;

            iovec_[write_index].iov_base =
                const_cast<uint8_t *>(queue[i].data());
            iovec_[write_index].iov_len = queue[i].size();
          }
        } else {
          const size_t aligned_size =
              iovec_[write_index].iov_len & (~(kSector - 1));
          VLOG(1) << "Was aligned, writing last chunk rounded from "
                  << queue[i].size() << " to " << aligned_size;
          iovec_[write_index].iov_len = aligned_size;

          const auto code = WriteV(iovec_.data(), write_index + 1, true,
                                   counted_size + aligned_size);
          if (code == WriteCode::kOutOfSpace) {
            return {
                .code = code,
                .messages_written = i,
            };
          }

          // Now, everything before here has been written.  Make an iovec out of
          // the rest and keep going.
          write_index = 0;
          counted_size = 0;

          iovec_[write_index].iov_base =
              const_cast<uint8_t *>(queue[i].data() + aligned_size);
          iovec_[write_index].iov_len = queue[i].size() - aligned_size;
        }
      }
      was_aligned = false;
    }
    VLOG(1) << "Writing " << iovec_[write_index].iov_len << " to "
            << filename();
    counted_size += iovec_[write_index].iov_len;
    ++write_index;
  }

  // Either write the aligned data if it is all aligned, or write the rest
  // unaligned if we wrote aligned up above.
  const auto code = WriteV(iovec_.data(), write_index, was_aligned, counted_size);
  return {
      .code = code,
      .messages_written = queue.size(),
  };
}

WriteCode FileHandler::WriteV(struct iovec *iovec_data, size_t iovec_size,
                              bool aligned, size_t counted_size) {
  // Configure the file descriptor to match the mode we should be in.  This is
  // safe to over-call since it only does the syscall if needed.
  if (aligned) {
    EnableDirect();
  } else {
    DisableDirect();
  }

  CHECK_GT(iovec_size, 0u);
  const auto start = aos::monotonic_clock::now();

  if (aligned) {
    CHECK_EQ((total_write_bytes_ % FileHandler::kSector), 0u)
        << ": Failed after writing " << total_write_bytes_
        << " to the file, attempting aligned write with unaligned start.";
    for (size_t i = 0; i < iovec_size; ++i) {
      absl::Span<const uint8_t> data(
          reinterpret_cast<const uint8_t *>(iovec_data[i].iov_base),
          iovec_data[i].iov_len);
      VLOG(2) << "  iov_base " << static_cast<void *>(iovec_data[i].iov_base)
              << ", iov_len " << iovec_data[i].iov_len;
      CHECK(IsAlignedStart(data) && IsAlignedLength(data));
      CHECK_GT(data.size(), 0u);
    }
  } else {
    size_t accumulated_write_bytes = total_write_bytes_;
    for (size_t i = 0; i < iovec_size; ++i) {
      absl::Span<const uint8_t> data(
          reinterpret_cast<const uint8_t *>(iovec_data[i].iov_base),
          iovec_data[i].iov_len);
      VLOG(2) << "  accumulated_write_bytes 0x" << std::hex
              << accumulated_write_bytes << " (" << std::dec
              << accumulated_write_bytes << "), iov_base "
              << static_cast<void *>(iovec_data[i].iov_base) << ", iov_len 0x"
              << std::hex << iovec_data[i].iov_len << " (" << std::dec
              << iovec_data[i].iov_len << ")";

      if ((accumulated_write_bytes % FileHandler::kSector) == 0) {
        CHECK(!IsAlignedStart(data) || !IsAlignedLength(data));
      }

      accumulated_write_bytes += data.size();
      CHECK_GT(data.size(), 0u);
    }
  }

  if (VLOG_IS_ON(2)) {
    size_t to_be_written = 0;
    for (size_t i = 0; i < iovec_size; ++i) {
      to_be_written += iovec_data[i].iov_len;
    }
    VLOG(2) << "Going to write " << to_be_written;
    CHECK_GT(to_be_written, 0u);
  }

  const ssize_t written = writev(fd_, iovec_data, iovec_size);
  VLOG(2) << "Wrote " << written << ", for iovec size " << iovec_size;

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

  total_write_bytes_ += written;
  write_stats_.UpdateStats(end - start, written, iovec_size);
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

std::unique_ptr<FileHandler> FileBackend::RequestFile(std::string_view id) {
  const std::string filename = absl::StrCat(base_name_, separator_, id);
  return std::make_unique<FileHandler>(filename);
}

RenamableFileBackend::RenamableFileBackend(std::string_view base_name)
    : base_name_(base_name), separator_(base_name_.back() == '/' ? "" : "_") {}

std::unique_ptr<FileHandler> RenamableFileBackend::RequestFile(
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
