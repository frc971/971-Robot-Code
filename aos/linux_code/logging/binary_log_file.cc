#include "aos/linux_code/logging/binary_log_file.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <setjmp.h>

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
  bool r = offset_ == static_cast<off_t>(info.st_size - kPageSize);
  is_last_page_ = r ? 2 : 1;
  return r;
}

void LogFileAccessor::MapNextPage() {
  if (writable_) {
    if (ftruncate(fd_, offset_ + kPageSize) == -1) {
      LOG(FATAL, "ftruncate(%d, %zd) failed with %d: %s. aborting\n", fd_,
          kPageSize, errno, strerror(errno));
    }
  }
  current_ = static_cast<char *>(
      mmap(NULL, kPageSize, PROT_READ | (writable_ ? PROT_WRITE : 0),
           MAP_SHARED, fd_, offset_));
  if (current_ == MAP_FAILED) {
    LOG(FATAL,
        "mmap(NULL, %zd, PROT_READ [ | PROT_WRITE], MAP_SHARED, %d, %jd)"
        " failed with %d: %s\n",
        kPageSize, fd_, static_cast<intmax_t>(offset_), errno,
        strerror(errno));
  }
  if (madvise(current_, kPageSize, MADV_SEQUENTIAL | MADV_WILLNEED) == -1) {
    LOG(WARNING, "madvise(%p, %zd, MADV_SEQUENTIAL | MADV_WILLNEED)"
                 " failed with %d: %s\n",
        current_, kPageSize, errno, strerror(errno));
  }
  offset_ += kPageSize;
}

void LogFileAccessor::Unmap(void *location) {
  if (munmap(location, kPageSize) == -1) {
    LOG(FATAL, "munmap(%p, %zd) failed with %d: %s. aborting\n", location,
        kPageSize, errno, strerror(errno));
  }
  is_last_page_ = 0;
  position_ = 0;
}

const LogFileMessageHeader *LogFileReader::ReadNextMessage(bool wait) {
  LogFileMessageHeader *r;
  do {
    r = static_cast<LogFileMessageHeader *>(
        static_cast<void *>(&current()[position()]));
    if (wait) {
      if (futex_wait(&r->marker) != 0) continue;
    }
    if (r->marker == 2) {
      Unmap(current());
      MapNextPage();
      CheckCurrentPageReadable();
      r = static_cast<LogFileMessageHeader *>(static_cast<void *>(current()));
    }
  } while (wait && r->marker == 0);
  if (r->marker == 0) {
    return NULL;
  }
  IncrementPosition(sizeof(LogFileMessageHeader) + r->name_size +
                    r->message_size);
  if (position() >= kPageSize) {
    // It's a lot better to blow up here rather than getting SIGBUS errors the
    // next time we try to read...
    LOG(FATAL, "corrupt log file running over page size\n");
  }
  return r;
}

// This mess is because the only not completely hackish way to do this is to set
// up a handler for SIGBUS/SIGSEGV that siglongjmps out to avoid either the
// instruction being repeated infinitely (and more signals being delivered) or
// (with SA_RESETHAND) the signal killing the process.
namespace {

void *volatile fault_address;
sigjmp_buf jump_context;

void CheckCurrentPageReadableHandler(int /*signal*/, siginfo_t *info, void *) {
  fault_address = info->si_addr;

  siglongjmp(jump_context, 1);
}

}  // namespace
void LogFileReader::CheckCurrentPageReadable() {
  if (sigsetjmp(jump_context, 1) == 0) {
    struct sigaction action;
    action.sa_sigaction = CheckCurrentPageReadableHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESETHAND | SA_SIGINFO;
    struct sigaction previous_bus, previous_segv;
    if (sigaction(SIGBUS, &action, &previous_bus) == -1) {
      LOG(FATAL, "sigaction(SIGBUS(=%d), %p, %p) failed with %d: %s\n",
          SIGBUS, &action, &previous_bus, errno, strerror(errno));
    }
    if (sigaction(SIGSEGV, &action, &previous_segv) == -1) {
      LOG(FATAL, "sigaction(SIGSEGV(=%d), %p, %p) failed with %d: %s\n",
          SIGSEGV, &action, &previous_segv, errno, strerror(errno));
    }

    char __attribute__((unused)) c = current()[0];

    if (sigaction(SIGBUS, &previous_bus, NULL) == -1) {
      LOG(FATAL, "sigaction(SIGBUS(=%d), %p, NULL) failed with %d: %s\n",
          SIGBUS, &previous_bus, errno, strerror(errno));
    }
    if (sigaction(SIGSEGV, &previous_segv, NULL) == -1) {
      LOG(FATAL, "sigaction(SIGSEGV(=%d), %p, NULL) failed with %d: %s\n",
          SIGSEGV, &previous_segv, errno, strerror(errno));
    }
  } else {
    if (fault_address == current()) {
      LOG(FATAL, "could not read 1 byte at offset 0x%jx into log file\n",
          static_cast<uintmax_t>(offset()));
    } else {
      LOG(FATAL, "faulted at %p, not %p like we were (maybe) supposed to\n",
          fault_address, current());
    }
  }
}

LogFileMessageHeader *LogFileWriter::GetWritePosition(size_t message_size) {
  if (position() + message_size + (kAlignment - (message_size % kAlignment)) +
      sizeof(mutex) > kPageSize) {
    char *const temp = current();
    MapNextPage();
    if (futex_set_value(static_cast<mutex *>(static_cast<void *>(
                    &temp[position()])), 2) == -1) {
      LOG(WARNING,
          "futex_set_value(%p, 2) failed with %d: %s. readers will hang\n",
          &temp[position()], errno, strerror(errno));
    }
    Unmap(temp);
  }
  LogFileMessageHeader *const r = static_cast<LogFileMessageHeader *>(
      static_cast<void *>(&current()[position()]));
  IncrementPosition(message_size);
  return r;
}

}  // namespace linux_code
}  // namespace logging
}  // namespace aos
