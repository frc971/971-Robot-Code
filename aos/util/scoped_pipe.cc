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

std::tuple<ScopedPipe::ScopedReadPipe, ScopedPipe::ScopedWritePipe>
ScopedPipe::MakePipe() {
  int fds[2];
  PCHECK(pipe(fds) != -1);
  PCHECK(fcntl(fds[0], F_SETFL, fcntl(fds[0], F_GETFL) | O_NONBLOCK) != -1);
  PCHECK(fcntl(fds[1], F_SETFL, fcntl(fds[1], F_GETFL) | O_NONBLOCK) != -1);
  return {ScopedReadPipe(fds[0]), ScopedWritePipe(fds[1])};
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
  CHECK(result == sizeof(data));
}

}  // namespace aos::util
