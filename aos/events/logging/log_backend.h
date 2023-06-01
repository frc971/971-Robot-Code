#ifndef AOS_EVENTS_LOGGING_LOG_BACKEND_H_
#define AOS_EVENTS_LOGGING_LOG_BACKEND_H_

#include <fcntl.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "aos/events/logging/buffer_encoder.h"
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

// Interface that abstract writing to log from media.
class LogSink {
 public:
  LogSink() = default;
  virtual ~LogSink() = default;

  LogSink(const LogSink &) = delete;
  LogSink &operator=(const LogSink &) = delete;

  // Try to open file. App will crash if there are other than out-of-space
  // problems with backend media.
  virtual WriteCode OpenForWrite() = 0;

  // Close the file handler.
  virtual WriteCode Close() = 0;

  // Returns true if sink is open and need to be closed.
  virtual bool is_open() const = 0;

  // Peeks messages from queue and writes it to file. Returns code when
  // out-of-space problem occurred along with number of messages from queue that
  // was written.
  virtual WriteResult Write(
      const absl::Span<const absl::Span<const uint8_t>> &queue) = 0;

  // Get access to statistics related to the write operations.
  WriteStats *WriteStatistics() { return &write_stats_; }

  // Name of the log sink.
  virtual std::string_view name() const = 0;

 private:
  WriteStats write_stats_;
};

// Source for iovec with an additional flag that pointer and size of data is
// aligned and be ready for O_DIRECT operation.
struct AlignedIovec {
  const uint8_t *data;
  size_t size;
  bool aligned;

  AlignedIovec(const uint8_t *data, size_t size, bool aligned)
      : data(data), size(size), aligned(aligned) {}
};

// Converts queue of pieces to write to the disk to the queue where every
// element is either aligned for O_DIRECT operation or marked as not aligned.
class QueueAligner {
 public:
  QueueAligner();

  // Reads input queue and fills with aligned and unaligned pieces. It is easy
  // to deal with smaller pieces and batch it during the write operation.
  void FillAlignedQueue(
      const absl::Span<const absl::Span<const uint8_t>> &queue);

  const std::vector<AlignedIovec> &aligned_queue() const {
    return aligned_queue_;
  }

 private:
  std::vector<AlignedIovec> aligned_queue_;
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
class FileHandler : public LogSink {
 public:
  // Size of an aligned sector used to detect when the data is aligned enough to
  // use O_DIRECT instead.
  static constexpr size_t kSector = 512u;

  explicit FileHandler(std::string filename, bool supports_odirect);
  ~FileHandler() override;

  FileHandler(const FileHandler &) = delete;
  FileHandler &operator=(const FileHandler &) = delete;

  // Try to open file. App will crash if there are other than out-of-space
  // problems with backend media.
  WriteCode OpenForWrite() override;

  // Close the file handler.
  WriteCode Close() override;

  // This will be true until Close() is called, unless the file couldn't be
  // created due to running out of space.
  bool is_open() const override { return fd_ != -1; }

  // Peeks messages from queue and writes it to file. Returns code when
  // out-of-space problem occurred along with number of messages from queue that
  // was written.
  //
  // The spans can be aligned or not, and can have any lengths.  This code will
  // write faster if the spans passed in start at aligned addresses, and are
  // multiples of kSector long (and the data written so far is also a multiple
  // of kSector length).
  WriteResult Write(
      const absl::Span<const absl::Span<const uint8_t>> &queue) override;

  // Name of the log sink mostly for informational purposes.
  std::string_view name() const override { return filename_; }

  // Number of bytes written in aligned mode. It is mostly for testing.
  size_t written_aligned() const { return written_aligned_; }

 protected:
  // This is used by subclasses who need to access filename.
  std::string_view filename() const { return filename_; }

 private:
  // Enables O_DIRECT on the open file if it is supported.  Cheap to call if it
  // is already enabled.
  void EnableDirect();
  // Disables O_DIRECT on the open file if it is supported.  Cheap to call if it
  // is already disabld.
  void DisableDirect();

  bool ODirectEnabled() const { return !!(flags_ & O_DIRECT); }

  // Writes a chunk of iovecs. aligned is true if all the data is kSector byte
  // aligned and multiples of it in length.
  WriteCode WriteV(const std::vector<struct iovec> &iovec, bool aligned);

  const std::string filename_;

  int fd_ = -1;

  // List of iovecs to use with writev.  This is a member variable to avoid
  // churn.
  std::vector<struct iovec> iovec_;

  QueueAligner queue_aligner_;

  int total_write_bytes_ = 0;
  int last_synced_bytes_ = 0;

  size_t written_aligned_ = 0;

  bool supports_odirect_ = true;
  int flags_ = 0;
};

// Interface to decouple log writing and media (file system or memory). It is
// handy to use for tests.
class LogBackend {
 public:
  virtual ~LogBackend() = default;

  // Request file-like object from the log backend. It maybe a file on a disk or
  // in memory. id is usually generated by log namer and looks like name of the
  // file within a log folder.
  virtual std::unique_ptr<LogSink> RequestFile(std::string_view id) = 0;
};

// Interface to decouple reading of logs and media (file system, memory or S3).
class LogSource {
 public:
  virtual ~LogSource() = default;

  // Provides a list of readable sources for log reading.
  virtual std::vector<std::string> ListFiles() const = 0;

  // Entry point for reading the content of log file.
  virtual std::unique_ptr<DataDecoder> GetDecoder(
      std::string_view id) const = 0;
};

// Implements requests log files from file system.
class FileBackend : public LogBackend, public LogSource {
 public:
  // base_name is the path to the folder where log files are.
  explicit FileBackend(std::string_view base_name, bool supports_direct);
  ~FileBackend() override = default;

  // Request file from a file system. It is not open yet.
  std::unique_ptr<LogSink> RequestFile(std::string_view id) override;

  // List all files that looks like log files under base_name.
  std::vector<std::string> ListFiles() const override;

  // Open decoder to read the content of the file.
  std::unique_ptr<DataDecoder> GetDecoder(std::string_view id) const override;

 private:
  const bool supports_odirect_;
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
    RenamableFileHandler(RenamableFileBackend *owner, std::string filename,
                         bool supports_odirect)
        : FileHandler(std::move(filename), supports_odirect), owner_(owner) {}
    ~RenamableFileHandler() final = default;

    // Closes and if needed renames file.
    WriteCode Close() final;

   private:
    RenamableFileBackend *owner_;
  };

  explicit RenamableFileBackend(std::string_view base_name,
                                bool supports_odirect);
  ~RenamableFileBackend() = default;

  // Request file from a file system. It is not open yet.
  std::unique_ptr<LogSink> RequestFile(std::string_view id) override;

  // TODO (Alexei): it is called by Logger, and left here for compatibility.
  // Logger should not call it.
  std::string_view base_name() const { return base_name_; }

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

  const bool supports_odirect_;
  std::string base_name_;
  std::string_view separator_;

  bool use_temp_files_ = false;
  std::string_view temp_suffix_;

  std::string old_base_name_;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOG_BACKEND_H_
