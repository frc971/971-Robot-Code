#ifndef AOS_EVENTS_LOGGING_LOG_BACKEND_H_
#define AOS_EVENTS_LOGGING_LOG_BACKEND_H_

#include <fcntl.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "aos/time/time.h"

namespace aos::logger {

class WriteStats {
 public:
  // The maximum time for a single write call, or 0 if none have been performed.
  std::chrono::nanoseconds max_write_time() const { return max_write_time_; }
  // The number of bytes in the longest write call, or -1 if none have been
  // performed.
  int max_write_time_bytes() const { return max_write_time_bytes_; }
  // The number of buffers in the longest write call, or -1 if none have been
  // performed.
  int max_write_time_messages() const { return max_write_time_messages_; }
  // The total time spent in write calls.
  std::chrono::nanoseconds total_write_time() const {
    return total_write_time_;
  }
  // The total number of writes which have been performed.
  int total_write_count() const { return total_write_count_; }
  // The total number of messages which have been written.
  int total_write_messages() const { return total_write_messages_; }
  // The total number of bytes which have been written.
  int total_write_bytes() const { return total_write_bytes_; }

  void ResetStats() {
    max_write_time_ = std::chrono::nanoseconds::zero();
    max_write_time_bytes_ = -1;
    max_write_time_messages_ = -1;
    total_write_time_ = std::chrono::nanoseconds::zero();
    total_write_count_ = 0;
    total_write_messages_ = 0;
    total_write_bytes_ = 0;
  }

  void UpdateStats(aos::monotonic_clock::duration duration, ssize_t written,
                   int iovec_size) {
    if (duration > max_write_time_) {
      max_write_time_ = duration;
      max_write_time_bytes_ = written;
      max_write_time_messages_ = iovec_size;
    }
    total_write_time_ += duration;
    ++total_write_count_;
    total_write_messages_ += iovec_size;
    total_write_bytes_ += written;
  }

 private:
  std::chrono::nanoseconds max_write_time_ = std::chrono::nanoseconds::zero();
  int max_write_time_bytes_ = -1;
  int max_write_time_messages_ = -1;
  std::chrono::nanoseconds total_write_time_ = std::chrono::nanoseconds::zero();
  int total_write_count_ = 0;
  int total_write_messages_ = 0;
  int total_write_bytes_ = 0;
};

// Currently, all write operations only cares about out-of-space error. This is
// a simple representation of write result.
enum class WriteCode { kOk, kOutOfSpace };

struct WriteResult {
  WriteCode code = WriteCode::kOk;
  size_t messages_written = 0;
};

// FileHandler is a replacement for bare filename in log writing and reading
// operations.
//
// There are a couple over-arching constraints on writing to keep track of.
//  1) The kernel is both faster and more efficient at writing large, aligned
//     chunks with O_DIRECT set on the file.  The alignment needed is specified
//     by kSector and is file system dependent.
//  2) Not all encoders support generating round multiples of kSector of data.
//     Rather than burden the API for detecting when that is the case, we want
//     DetachedBufferWriter to be as efficient as it can at writing what given.
//  3) Some files are small and not updated frequently.  They need to be
//     flushed or we will lose data on power off.  It is most efficient to write
//     as much as we can aligned by kSector and then fall back to the non direct
//     method when it has been flushed.
//  4) Not all filesystems support O_DIRECT, and different sizes may be optimal
//     for different machines.  The defaults should work decently anywhere and
//     be tunable for faster systems.
// TODO (Alexei): need 2 variations, to support systems with and without
// O_DIRECT
class FileHandler {
 public:
  // Size of an aligned sector used to detect when the data is aligned enough to
  // use O_DIRECT instead.
  static constexpr size_t kSector = 512u;

  explicit FileHandler(std::string filename);
  virtual ~FileHandler();

  FileHandler(const FileHandler &) = delete;
  FileHandler &operator=(const FileHandler &) = delete;

  // Try to open file. App will crash if there are other than out-of-space
  // problems with backend media.
  virtual WriteCode OpenForWrite();

  // Close the file handler.
  virtual WriteCode Close();

  // This will be true until Close() is called, unless the file couldn't be
  // created due to running out of space.
  bool is_open() const { return fd_ != -1; }

  // Peeks messages from queue and writes it to file. Returns code when
  // out-of-space problem occurred along with number of messages from queue that
  // was written.
  //
  // The spans can be aligned or not, and can have any lengths.  This code will
  // write faster if the spans passed in start at aligned addresses, and are
  // multiples of kSector long (and the data written so far is also a multiple
  // of kSector length).
  virtual WriteResult Write(
      const absl::Span<const absl::Span<const uint8_t>> &queue);

  // TODO (Alexei): it is rather leaked abstraction.
  // Path to the concrete log file.
  std::string_view filename() const { return filename_; }

  int fd() const { return fd_; }

  // Get access to statistics related to the write operations.
  WriteStats *WriteStatistics() { return &write_stats_; }

 private:
  // Enables O_DIRECT on the open file if it is supported.  Cheap to call if it
  // is already enabled.
  void EnableDirect();
  // Disables O_DIRECT on the open file if it is supported.  Cheap to call if it
  // is already disabld.
  void DisableDirect();

  bool ODirectEnabled() const { return !!(flags_ & O_DIRECT); }

  // Writes a chunk of iovecs.  aligned is true if all the data is kSector byte
  // aligned and multiples of it in length, and counted_size is the sum of the
  // sizes of all the chunks of data.
  WriteCode WriteV(struct iovec *iovec_data, size_t iovec_size, bool aligned,
                   size_t counted_size);

  const std::string filename_;

  int fd_ = -1;

  // List of iovecs to use with writev.  This is a member variable to avoid
  // churn.
  std::vector<struct iovec> iovec_;

  int total_write_bytes_ = 0;
  int last_synced_bytes_ = 0;

  bool supports_odirect_ = true;
  int flags_ = 0;

  WriteStats write_stats_;
};

// Class that decouples log writing and media (file system or memory). It is
// handy to use for tests.
class LogBackend {
 public:
  virtual ~LogBackend() = default;

  // Request file-like object from the log backend. It maybe a file on a disk or
  // in memory. id is usually generated by log namer and looks like name of the
  // file within a log folder.
  virtual std::unique_ptr<FileHandler> RequestFile(std::string_view id) = 0;
};

// Implements requests log files from file system.
class FileBackend : public LogBackend {
 public:
  // base_name is the path to the folder where log files are.
  explicit FileBackend(std::string_view base_name);
  ~FileBackend() override = default;

  // Request file from a file system. It is not open yet.
  std::unique_ptr<FileHandler> RequestFile(std::string_view id) override;

 private:
  const std::string base_name_;
  const std::string_view separator_;
};

// Provides a file backend that supports renaming of the base log folder and
// temporary files.
class RenamableFileBackend : public LogBackend {
 public:
  // Adds call to rename, when closed.
  class RenamableFileHandler final : public FileHandler {
   public:
    RenamableFileHandler(RenamableFileBackend *owner, std::string filename)
        : FileHandler(std::move(filename)), owner_(owner) {}
    ~RenamableFileHandler() final = default;

    // Returns false if not enough memory, true otherwise.
    WriteCode Close() final;

   private:
    RenamableFileBackend *owner_;
  };

  explicit RenamableFileBackend(std::string_view base_name);
  ~RenamableFileBackend() = default;

  // Request file from a file system. It is not open yet.
  std::unique_ptr<FileHandler> RequestFile(std::string_view id) override;

  // TODO (Alexei): it is called by Logger, and left here for compatibility.
  // Logger should not call it.
  std::string_view base_name() { return base_name_; }

  // If temp files are enabled, then this will write files with the .tmp
  // suffix, and then rename them to the desired name after they are fully
  // written.
  //
  // This is useful to enable incremental copying of the log files.
  //
  // Defaults to writing directly to the final filename.
  void EnableTempFiles();

  // Moves the current log location to the new name. Returns true if a change
  // was made, false otherwise.
  // Only renaming the folder is supported, not the file base name.
  bool RenameLogBase(std::string_view new_base_name);

 private:
  // This function called after file closed, to adjust file names in case of
  // base name was changed or temp files enabled.
  WriteCode RenameFileAfterClose(std::string_view filename);

  std::string base_name_;
  std::string_view separator_;

  bool use_temp_files_ = false;
  std::string_view temp_suffix_;

  std::string old_base_name_;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOG_BACKEND_H_
