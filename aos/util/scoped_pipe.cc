#include "aos/util/scoped_pipe.h"

#include <fcntl.h>
#include "glog/logging.h"

namespace aos::util {

ScopedPipe::ScopedPipe(int fd) : fd_(fd) {}

ScopedPipe::~ScopedPipe() {
  if (fd_ != -1) {
    PCHECK(close(fd_) != -1);
  }
}

ScopedPipe::ScopedPipe(ScopedPipe &&scoped_pipe) : fd_(scoped_pipe.fd_) {
  scoped_pipe.fd_ = -1;
}

ScopedPipe &ScopedPipe::operator=(ScopedPipe &&scoped_pipe) {
  if (fd_ != -1) {
    PCHECK(close(fd_) != -1);
  }
  fd_ = scoped_pipe.fd_;
  scoped_pipe.fd_ = -1;
  return *this;
}

ScopedPipe::PipePair ScopedPipe::MakePipe() {
  int fds[2];
  PCHECK(pipe(fds) != -1);
  PCHECK(fcntl(fds[0], F_SETFL, fcntl(fds[0], F_GETFL) | O_NONBLOCK) != -1);
  PCHECK(fcntl(fds[1], F_SETFL, fcntl(fds[1], F_GETFL) | O_NONBLOCK) != -1);
  return {std::unique_ptr<ScopedReadPipe>(new ScopedReadPipe(fds[0])),
          std::unique_ptr<ScopedWritePipe>(new ScopedWritePipe(fds[1]))};
}

void ScopedPipe::SetCloexec() {
  // FD_CLOEXEC is the only known file descriptor flag, but call GETFD just in
  // case.
  int flags = fcntl(fd(), F_GETFD);
  PCHECK(flags != -1);
  PCHECK(fcntl(fd(), F_SETFD, flags | FD_CLOEXEC) != -1);
}

size_t ScopedPipe::ScopedReadPipe::Read(std::string *buffer) {
  CHECK_NOTNULL(buffer);
  constexpr ssize_t kBufferSize = 1024;
  const size_t original_size = buffer->size();
  size_t read_bytes = 0;
  while (true) {
    buffer->resize(buffer->size() + kBufferSize);
    const ssize_t result =
        read(fd(), buffer->data() + buffer->size() - kBufferSize, kBufferSize);
    if (result == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        buffer->resize(original_size);
        return 0;
      }
      PLOG(FATAL) << "Error on reading pipe.";
    } else if (result < kBufferSize) {
      read_bytes += result;
      buffer->resize(original_size + read_bytes);
      break;
    } else {
      CHECK_EQ(result, kBufferSize);
      read_bytes += result;
    }
  }
  return read_bytes;
}

std::optional<uint32_t> ScopedPipe::ScopedReadPipe::Read() {
  uint32_t buf;
  ssize_t result = read(fd(), &buf, sizeof(buf));
  if (result == sizeof(buf)) {
    return buf;
  } else {
    return std::nullopt;
  }
}

void ScopedPipe::ScopedWritePipe::Write(uint32_t data) {
  ssize_t result = write(fd(), &data, sizeof(data));
  PCHECK(result != -1);
  CHECK_EQ(static_cast<size_t>(result), sizeof(data));
}

void ScopedPipe::ScopedWritePipe::Write(absl::Span<const uint8_t> data) {
  ssize_t result = write(fd(), data.data(), data.size());
  PCHECK(result != -1);
  CHECK_EQ(static_cast<size_t>(result), data.size());
}

}  // namespace aos::util
