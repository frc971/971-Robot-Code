#ifndef AOS_UTIL_SCOPED_PIPE_H_
#define AOS_UTIL_SCOPED_PIPE_H_

#include <stdint.h>

#include <memory>
#include <optional>

#include "absl/types/span.h"

namespace aos::util {

// RAII Pipe for sending individual ints between reader and writer.
class ScopedPipe {
 public:
  class ScopedReadPipe;
  class ScopedWritePipe;

  struct PipePair {
    std::unique_ptr<ScopedReadPipe> read;
    std::unique_ptr<ScopedWritePipe> write;
  };

  static PipePair MakePipe();

  virtual ~ScopedPipe();

  int fd() const { return fd_; }
  // Sets FD_CLOEXEC on the file descriptor.
  void SetCloexec();

 private:
  ScopedPipe(int fd = -1);

  int fd_;

  ScopedPipe(const ScopedPipe &) = delete;
  ScopedPipe &operator=(const ScopedPipe &) = delete;
  ScopedPipe(ScopedPipe &&);
  ScopedPipe &operator=(ScopedPipe &&);
};

class ScopedPipe::ScopedReadPipe : public ScopedPipe {
 public:
  std::optional<uint32_t> Read();
  // Reads as many bytes as possible out of the pipe, appending them to the end
  // of the provided buffer. Returns the number of bytes read. Dies on errors
  // other than EAGAIN or EWOULDBLOCK.
  size_t Read(std::string *buffer);

 private:
  using ScopedPipe::ScopedPipe;

  friend class ScopedPipe;
};

class ScopedPipe::ScopedWritePipe : public ScopedPipe {
 public:
  void Write(uint32_t data);
  // Writes the entirety of the specified buffer to the pipe. Dies on failure.
  void Write(absl::Span<const uint8_t> data);

 private:
  using ScopedPipe::ScopedPipe;

  friend class ScopedPipe;
};

}  //  namespace aos::util

#endif  // AOS_UTIL_SCOPED_PIPE_H_
