#ifndef AOS_LINUX_CODE_LOGGING_BINARY_LOG_FILE_H_
#define AOS_LINUX_CODE_LOGGING_BINARY_LOG_FILE_H_

#include <sys/types.h>
#include <stddef.h>
#include <stdint.h>

#include <algorithm>

#include "aos/common/logging/logging_impl.h"

namespace aos {
namespace logging {
namespace linux_code {

// What to align messages to. A macro because it gets used in attributes.
// This definition gets #undefed later. Use LogFileAccessor::kAlignment instead.
#define MESSAGE_ALIGNMENT 8

// File format: {
//   LogFileMessageHeader header;
//   char *name;  // of the process that wrote the message
//   void *message;
// } not crossing kPageSize boundaries into the file and aligned to
// MESSAGE_ALIGNMENT.
//
// Field sizes designed to fit the various values from LogMessage even on
// other machines (hopefully) because they're baked into the files. They are
// layed out so that all of the fields are aligned even though the whole thing
// is packed.
//
// A lot of the fields don't have comments because they're the same as the
// identically named fields in LogMessage.
struct __attribute__((aligned(MESSAGE_ALIGNMENT))) __attribute__((packed))
    LogFileMessageHeader {
  // Represents the type of an individual message.
  enum class MessageType : uint16_t {
    // char[] (no '\0' on the end).
    kString,
    kStructType,
    kStruct,
    kMatrix,
  };

  // Gets futex_set once this one has been written
  // for readers keeping up with a live writer.
  //
  // Gets initialized to 0 by ftruncate.
  //
  // There will be something here after the last message on a "page" set to 2
  // (by the futex_set) to indicate that the next message is on the next page.
  mutex marker;
  static_assert(sizeof(marker) == 4, "mutex changed size!");
  static_assert(MESSAGE_ALIGNMENT >= alignof(mutex),
                "MESSAGE_ALIGNMENT is too small");

  uint32_t time_sec;
  static_assert(sizeof(time_sec) >= sizeof(LogMessage::seconds),
                "tv_sec won't fit");
  uint32_t time_nsec;
  static_assert(sizeof(time_nsec) >= sizeof(LogMessage::nseconds),
                "tv_nsec won't fit");

  int32_t source;
  static_assert(sizeof(source) >= sizeof(LogMessage::source), "PIDs won't fit");

  // Both including all of the bytes in that part of the message.
  uint32_t name_size, message_size;

  uint16_t sequence;
  static_assert(sizeof(sequence) == sizeof(LogMessage::sequence),
                "something changed");

  MessageType type;

  log_level level;
  static_assert(sizeof(level) == 1, "log_level changed size!");
};
static_assert(std::is_pod<LogFileMessageHeader>::value,
              "LogFileMessageHeader will to get dumped to a file");
static_assert(offsetof(LogFileMessageHeader, marker) == 0,
              "marker has to be at the start so readers can find it");

// Handles the mmapping and munmapping for reading and writing log files.
class LogFileAccessor {
 public:
  LogFileAccessor(int fd, bool writable);
  ~LogFileAccessor() {
    if (use_read_ == Maybe::kYes) {
      delete[] current_;
    }
  }

  // Asynchronously syncs all open mappings.
  void Sync() const;

  bool IsLastPage();

 protected:
  // The size of the chunks that get mmaped/munmapped together. Large enough so
  // that not too much space is wasted and it's hopefully bigger than and a
  // multiple of the system page size but small enough so that really large
  // chunks of memory don't have to get mapped at the same time.
  static const size_t kPageSize = 16384;
  // What to align messages to, copied into an actual constant.
  static const size_t kAlignment = MESSAGE_ALIGNMENT;
#undef MESSAGE_ALIGNMENT

  char *current() const { return current_; }
  size_t position() const { return position_; }
  off_t offset() const { return offset_; }

  void IncrementPosition(size_t size) {
    position_ += size;
    AlignPosition();
  }

  void MapNextPage();
  void Unmap(void *location);

  // Advances position to the next (aligned) location.
  void AlignPosition() {
    position_ += kAlignment - (position_ % kAlignment);
  }

 protected:
  bool definitely_use_read() const { return use_read_ == Maybe::kYes; }
  bool definitely_use_mmap() const { return use_read_ == Maybe::kNo; }

 private:
  // Used for representing things that we might know to be true/false or we
  // might not know (yet).
  enum class Maybe { kUnknown, kYes, kNo };

  const int fd_;
  const bool writable_;

  // Into the file. Always a multiple of kPageSize.
  off_t offset_;
  char *current_;
  size_t position_;

  Maybe is_last_page_ = Maybe::kUnknown;

  // Use read instead of mmap (necessary for fds that don't support mmap).
  Maybe use_read_ = Maybe::kUnknown;
};

class LogFileReader : public LogFileAccessor {
 public:
  LogFileReader(int fd) : LogFileAccessor(fd, false) {}

  // May return NULL iff wait is false.
  const LogFileMessageHeader *ReadNextMessage(bool wait);

 private:
  // Tries reading from the current page to see if it fails because the file
  // isn't big enough.
  void CheckCurrentPageReadable();
};

class LogFileWriter : public LogFileAccessor {
 public:
  LogFileWriter(int fd) : LogFileAccessor(fd, true) {}

  // message_size should be the total number of bytes needed for the message.
  LogFileMessageHeader *GetWritePosition(size_t message_size);
};

}  // namespace linux_code
}  // namespace logging
}  // namespace aos

#endif  // AOS_LINUX_CODE_LOGGING_BINARY_LOG_FILE_H_
