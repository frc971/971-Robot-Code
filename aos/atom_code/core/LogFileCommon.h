#ifndef AOS_ATOM_CODE_CORE_LOG_FILE_COMMON_H_
#define AOS_ATOM_CODE_CORE_LOG_FILE_COMMON_H_

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>

#include "aos/aos_core.h"

namespace aos {

// File format: {
//   LogFileMessageHeader header;
//   char *name; // of the process that wrote the message
//   char *message;
// } not crossing kPageSize boundaries into the file.
//
// Field sizes designed to fit the various values from log_message even on
// other machines (hopefully) because they're baked into the files.
struct __attribute__((aligned)) LogFileMessageHeader {
  // gets condition_set once this one has been written
  // for readers keeping up with a live writer
  //
  // gets initialized to 0 by ftruncate
  // 
  // there will be something here after the last log on a "page" set to 2
  // (by the condition_set) to indicate that the next log is on the next page
  mutex marker;
  static_assert(sizeof(marker) == 4, "mutex changed size!");
  log_level level;
  static_assert(sizeof(level) == 1, "log_level changed size!");

  uint64_t time_sec;
  static_assert(sizeof(time_sec) >= sizeof(log_message::time.tv_sec), "tv_sec won't fit");
  uint64_t time_nsec;
  static_assert(sizeof(time_nsec) >= sizeof(log_message::time.tv_nsec),
                "tv_nsec won't fit");

  int32_t source; // pid or -1 for crio
  static_assert(sizeof(source) >= sizeof(log_message::source), "PIDs won't fit");
  uint8_t sequence;
  static_assert(sizeof(sequence) == sizeof(log_crio_message::sequence),
                "something changed");
  static_assert(sizeof(sequence) == sizeof(log_message::sequence),
                "something changed");

  // both including the terminating '\0'
  uint32_t name_size;
  uint32_t message_size;
};
static_assert(std::is_pod<LogFileMessageHeader>::value,
              "LogFileMessageHeader will to get dumped to a file");

// Handles the mmapping and munmapping for reading and writing log files.
class LogFileAccessor {
 private:
  // The size of the chunks that get mmaped/munmapped together. Large enough so
  // that not too much space is wasted and it's hopefully bigger than and a
  // multiple of the system page size but small enough so that really large chunks
  // of memory don't have to get mapped at the same time.
  static const size_t kPageSize = 32768;
  // What to align messages to. Necessary for futexes to work.
  static const size_t kAlignment = 64;
  static_assert(kAlignment >= __alignof__(mutex), "futexes will complain");

  const int fd_;
  const bool writable_;

  off_t offset_; // into the file. will be aligned to kPageSize
  char *current_;
  size_t position_;

  inline unsigned long SystemPageSize() {
    static unsigned long r = sysconf(_SC_PAGESIZE);
    return r;
  }
  void MapNextPage() {
    if (writable_) {
      if (ftruncate(fd_, offset_ + kPageSize) == -1) {
        fprintf(stderr, "ftruncate(%d, %zd) failed with %d: %s. aborting\n",
                fd_, kPageSize, errno, strerror(errno));
        printf("see stderr\n");
        abort();
      }
    }
    current_ = static_cast<char *>(mmap(NULL, kPageSize,
                                        PROT_READ | (writable_ ? PROT_WRITE : 0),
                                        MAP_SHARED, fd_, offset_));
    if (current_ == MAP_FAILED) {
      fprintf(stderr, "mmap(NULL, %zd, PROT_READ | PROT_WRITE, MAP_SHARED, %d, %jd)"
              " failed with %d: %s. aborting\n", kPageSize, fd_,
              static_cast<intmax_t>(offset_), errno, strerror(errno));
      printf("see stderr\n");
      abort();
    }
    offset_ += kPageSize;
  }
  void Unmap(void *location) {
    if (munmap(location, kPageSize) == -1) {
      fprintf(stderr, "munmap(%p, %zd) failed with %d: %s. aborting\n",
              location, kPageSize, errno, strerror(errno));
      printf("see stderr\n");
      abort();
    }
  }
 public:
  LogFileAccessor(int fd, bool writable) : fd_(fd), writable_(writable),
    offset_(0), current_(0), position_(0) {
    // check to make sure that mmap will allow mmaping in chunks of kPageSize
    if (SystemPageSize() > kPageSize || (kPageSize % SystemPageSize()) != 0) {
      fprintf(stderr, "LogFileCommon: system page size (%lu)"
              " not compatible with kPageSize (%zd). aborting\n",
              SystemPageSize(), kPageSize);
      printf("see stderr\n");
      abort();
    }

    MapNextPage();
  }
  // message_size should be the total number of bytes needed for the message
  LogFileMessageHeader *GetWritePosition(size_t message_size) {
    if (position_ + message_size + (kAlignment - (message_size % kAlignment)) +
        sizeof(mutex) > kPageSize) {
      char *const temp = current_;
      MapNextPage();
      if (condition_set_value(reinterpret_cast<mutex *>(&temp[position_]), 2) != 0) {
        fprintf(stderr, "LogFileCommon: condition_set_value(%p, 2) failed with %d: %s."
                " readers will hang\n", &temp[position_], errno, strerror(errno));
      }
      Unmap(temp);
      position_ = 0;
    }
    LogFileMessageHeader *const r = reinterpret_cast<LogFileMessageHeader *>(
        &current_[position_]);
    position_ += message_size;
    // keep it aligned for next time
    position_ += kAlignment - (position_ % kAlignment);
    return r;
  }
  // may only return NULL if wait is false
  const LogFileMessageHeader *ReadNextMessage(bool wait) {
    LogFileMessageHeader *r;
    do {
      r = reinterpret_cast<LogFileMessageHeader *>(&current_[position_]);
      if (wait) {
        if (condition_wait(&r->marker) != 0) continue;
      }
      if (r->marker == 2) {
        Unmap(current_);
        MapNextPage();
        position_ = 0;
        r = reinterpret_cast<LogFileMessageHeader *>(current_);
      }
    } while (wait && r->marker == 0);
    if (r->marker == 0) {
      return NULL;
    }
    position_ += sizeof(LogFileMessageHeader) + r->name_size + r->message_size;
    // keep it aligned for next time
    position_ += kAlignment - (position_ % kAlignment);
    return r;
  }

  // asynchronously syncs all open mappings
  void Sync() {
    msync(current_, kPageSize, MS_ASYNC | MS_INVALIDATE);
  }

  void MoveToEnd() {
    Unmap(current_);
    struct stat info;
    if (fstat(fd_, &info) == -1) {
      fprintf(stderr, "LOgFileCommon: fstat(%d, %p) failed with %d: %s\n",
              fd_, &info, errno, strerror(errno));
      printf("see stderr\n");
      abort();
    }
    offset_ = info.st_size - kPageSize;
    MapNextPage();
  }
};

};

#endif

