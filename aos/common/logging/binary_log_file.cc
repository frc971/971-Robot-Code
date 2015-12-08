#include "aos/common/logging/binary_log_file.h"

#include <stdio.h>
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

void LogFileAccessor::SkipToLastSeekablePage() {
  CHECK(definitely_use_mmap());

  struct stat info;
  if (fstat(fd_, &info) == -1) {
    PLOG(FATAL, "fstat(%d, %p) failed", fd_, &info);
  }

  CHECK((info.st_size % kPageSize) == 0);
  const auto last_readable_page_number = (info.st_size / kPageSize) - 1;
  const auto last_seekable_page_number =
      last_readable_page_number / kSeekPages * kSeekPages;
  const off_t new_offset = last_seekable_page_number * kPageSize;
  // We don't want to go backwards...
  if (new_offset > offset_) {
    Unmap(current_);
    offset_ = new_offset;
    MapNextPage();
  }
}

// The only way to tell is using fstat, but we don't really want to be making a
// syscall every single time somebody wants to know the answer, so it gets
// cached in is_last_page_.
bool LogFileAccessor::IsLastPage() {
  if (is_last_page_ != Maybe::kUnknown) {
    return is_last_page_ == Maybe::kYes;
  }

  struct stat info;
  if (fstat(fd_, &info) == -1) {
    PLOG(FATAL, "fstat(%d, %p) failed", fd_, &info);
  }
  bool r = offset_ == static_cast<off_t>(info.st_size - kPageSize);
  is_last_page_ = r ? Maybe::kYes : Maybe::kNo;
  return r;
}

void LogFileAccessor::MapNextPage() {
  if (writable_) {
    if (ftruncate(fd_, offset_ + kPageSize) == -1) {
      PLOG(FATAL, "ftruncate(%d, %zd) failed", fd_, kPageSize);
    }
  }

  if (use_read_ == Maybe::kYes) {
    ssize_t todo = kPageSize;
    while (todo > 0) {
      ssize_t result = read(fd_, current_ + (kPageSize - todo), todo);
      if (result == -1) {
        PLOG(FATAL, "read(%d, %p, %zu) failed", fd_,
             current_ + (kPageSize - todo), todo);
      } else if (result == 0) {
        memset(current_, 0, todo);
        result = todo;
      }
      todo -= result;
    }
    CHECK_EQ(0, todo);
  } else {
    current_ = static_cast<char *>(
        mmap(NULL, kPageSize, PROT_READ | (writable_ ? PROT_WRITE : 0),
             MAP_SHARED, fd_, offset_));
    if (current_ == MAP_FAILED) {
      if (!writable_ && use_read_ == Maybe::kUnknown && errno == ENODEV) {
        LOG(INFO, "Falling back to reading the file using read(2).\n");
        use_read_ = Maybe::kYes;
        current_ = new char[kPageSize];
        MapNextPage();
        return;
      } else {
        PLOG(FATAL,
             "mmap(NULL, %zd, PROT_READ [ | PROT_WRITE], MAP_SHARED, %d, %jd)"
             " failed",
             kPageSize, fd_, static_cast<intmax_t>(offset_));
      }
    } else {
      use_read_ = Maybe::kNo;
    }
    if (madvise(current_, kPageSize, MADV_SEQUENTIAL | MADV_WILLNEED) == -1) {
      PLOG(WARNING, "madvise(%p, %zd, MADV_SEQUENTIAL | MADV_WILLNEED) failed",
           current_, kPageSize);
    }
  }
  offset_ += kPageSize;
}

void LogFileAccessor::Unmap(void *location) {
  CHECK_NE(Maybe::kUnknown, use_read_);

  if (use_read_ == Maybe::kNo) {
    if (munmap(location, kPageSize) == -1) {
      PLOG(FATAL, "munmap(%p, %zd) failed", location, kPageSize);
    }
  }
  is_last_page_ = Maybe::kUnknown;
  position_ = 0;
}

const LogFileMessageHeader *LogFileReader::ReadNextMessage(bool wait) {
  LogFileMessageHeader *r;
  do {
    r = static_cast<LogFileMessageHeader *>(
        static_cast<void *>(&current()[position()]));
    if (wait) {
      CHECK(definitely_use_mmap());
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
  if (definitely_use_read()) return;

  if (sigsetjmp(jump_context, 1) == 0) {
    struct sigaction action;
    action.sa_sigaction = CheckCurrentPageReadableHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESETHAND | SA_SIGINFO;
    struct sigaction previous_bus, previous_segv;
    if (sigaction(SIGBUS, &action, &previous_bus) == -1) {
      PLOG(FATAL, "sigaction(SIGBUS(=%d), %p, %p) failed",
           SIGBUS, &action, &previous_bus);
    }
    if (sigaction(SIGSEGV, &action, &previous_segv) == -1) {
      PLOG(FATAL, "sigaction(SIGSEGV(=%d), %p, %p) failed",
           SIGSEGV, &action, &previous_segv);
    }

    char __attribute__((unused)) c = current()[0];

    if (sigaction(SIGBUS, &previous_bus, NULL) == -1) {
      PLOG(FATAL, "sigaction(SIGBUS(=%d), %p, NULL) failed",
           SIGBUS, &previous_bus);
    }
    if (sigaction(SIGSEGV, &previous_segv, NULL) == -1) {
      PLOG(FATAL, "sigaction(SIGSEGV(=%d), %p, NULL) failed",
           SIGSEGV, &previous_segv);
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
  if (NeedNewPageFor(message_size)) ForceNewPage();
  LogFileMessageHeader *const r = static_cast<LogFileMessageHeader *>(
      static_cast<void *>(&current()[position()]));
  IncrementPosition(message_size);
  return r;
}

// A number of seekable pages, not the actual file offset, is stored in *cookie.
bool LogFileWriter::ShouldClearSeekableData(off_t *cookie,
                                            size_t next_message_size) const {
  off_t next_message_page = (offset() / kPageSize) - 1;
  if (NeedNewPageFor(next_message_size)) {
    ++next_message_page;
  }
  const off_t current_seekable_page = next_message_page / kSeekPages;
  CHECK_LE(*cookie, current_seekable_page);
  const bool r = *cookie != current_seekable_page;
  *cookie = current_seekable_page;
  return r;
}

bool LogFileWriter::NeedNewPageFor(size_t bytes) const {
  return position() + bytes + (kAlignment - (bytes % kAlignment)) +
             sizeof(aos_futex) >
         kPageSize;
}

void LogFileWriter::ForceNewPage() {
  char *const temp = current();
  MapNextPage();
  if (futex_set_value(
          static_cast<aos_futex *>(static_cast<void *>(&temp[position()])),
          2) == -1) {
    PLOG(WARNING, "readers will hang because futex_set_value(%p, 2) failed",
         &temp[position()]);
  }
  Unmap(temp);
}

}  // namespace linux_code
}  // namespace logging
}  // namespace aos
