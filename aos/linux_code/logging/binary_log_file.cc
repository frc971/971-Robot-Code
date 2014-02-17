#include "aos/linux_code/logging/binary_log_file.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace aos {
namespace logging {
namespace linux_code {
namespace {

unsigned long SystemPageSize() {
  static unsigned long r = sysconf(_SC_PAGESIZE);
  return r;
}

}  // namespace

LogFileAccessor::LogFileAccessor(int fd, bool writable)
    : fd_(fd), writable_(writable), offset_(0), current_(0), position_(0) {
  // Check to make sure that mmap will allow mmaping in chunks of kPageSize.
  if (SystemPageSize() > kPageSize || (kPageSize % SystemPageSize()) != 0) {
    LOG(FATAL, "system page size (%lu) not factor of kPageSize (%zd).\n",
        SystemPageSize(), kPageSize);
  }

  MapNextPage();
}

LogFileMessageHeader *LogFileAccessor::GetWritePosition(size_t message_size) {
  if (position_ + message_size + (kAlignment - (message_size % kAlignment)) +
      sizeof(mutex) > kPageSize) {
    char *const temp = current_;
    MapNextPage();
    if (futex_set_value(static_cast<mutex *>(static_cast<void *>(
                    &temp[position_])), 2) == -1) {
      LOG(WARNING,
          "futex_set_value(%p, 2) failed with %d: %s. readers will hang\n",
          &temp[position_], errno, strerror(errno));
    }
    Unmap(temp);
    position_ = 0;
  }
  LogFileMessageHeader *const r = static_cast<LogFileMessageHeader *>(
      static_cast<void *>(&current_[position_]));
  position_ += message_size;
  AlignPosition();
  return r;
}

const LogFileMessageHeader *LogFileAccessor::ReadNextMessage(bool wait) {
  LogFileMessageHeader *r;
  do {
    r = static_cast<LogFileMessageHeader *>(
        static_cast<void *>(&current_[position_]));
    if (wait) {
      if (futex_wait(&r->marker) != 0) continue;
    }
    if (r->marker == 2) {
      Unmap(current_);
      MapNextPage();
      position_ = 0;
      r = static_cast<LogFileMessageHeader *>(static_cast<void *>(current_));
    }
  } while (wait && r->marker == 0);
  if (r->marker == 0) {
    return NULL;
  }
  position_ += sizeof(LogFileMessageHeader) + r->name_size + r->message_size;
  AlignPosition();
  if (position_ >= kPageSize) {
    LOG(FATAL, "corrupt log file running over page size\n");
  }
  return r;
}

void LogFileAccessor::Sync() const {
  msync(current_, kPageSize, MS_ASYNC | MS_INVALIDATE);
}

bool LogFileAccessor::IsLastPage() {
  if (is_last_page_ != 0) {
    return is_last_page_ == 2;
  }

  struct stat info;
  if (fstat(fd_, &info) == -1) {
    LOG(FATAL, "fstat(%d, %p) failed with %d: %s\n", fd_, &info, errno,
        strerror(errno));
  }
  bool r = offset_ == info.st_size - kPageSize;
  is_last_page_ = r ? 2 : 1;
  return r;
}

void LogFileAccessor::MapNextPage() {
  if (writable_) {
    if (ftruncate(fd_, offset_ + kPageSize) == -1) {
      fprintf(stderr, "ftruncate(%d, %zd) failed with %d: %s. aborting\n",
              fd_, kPageSize, errno, strerror(errno));
      printf("see stderr\n");
      abort();
    }
  }
  current_ = static_cast<char *>(
      mmap(NULL, kPageSize, PROT_READ | (writable_ ? PROT_WRITE : 0),
           MAP_SHARED, fd_, offset_));
  if (current_ == MAP_FAILED) {
    LOG(FATAL,
        "mmap(NULL, %zd, PROT_READ | PROT_WRITE, MAP_SHARED, %d, %jd)"
        " failed with %d: %s. aborting\n",
        kPageSize, fd_, static_cast<intmax_t>(offset_), errno,
        strerror(errno));
  }
  offset_ += kPageSize;
}

void LogFileAccessor::Unmap(void *location) {
  if (munmap(location, kPageSize) == -1) {
    LOG(FATAL, "munmap(%p, %zd) failed with %d: %s. aborting\n", location,
        kPageSize, errno, strerror(errno));
  }
  is_last_page_ = 0;
}

}  // namespace linux_code
}  // namespace logging
}  // namespace aos
