#ifndef AOS_UTIL_SCOPED_PIPE_H_
#define AOS_UTIL_SCOPED_PIPE_H_

#include <stdint.h>

#include <optional>
#include <tuple>

namespace aos::util {

// RAII Pipe for sending individual ints between reader and writer.
class ScopedPipe {
 public:
  class ScopedReadPipe;
  class ScopedWritePipe;

  static std::tuple<ScopedReadPipe, ScopedWritePipe> MakePipe();

  virtual ~ScopedPipe();

  int fd() const { return fd_; }

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

 private:
  using ScopedPipe::ScopedPipe;

  friend class ScopedPipe;
};

class ScopedPipe::ScopedWritePipe : public ScopedPipe {
 public:
  void Write(uint32_t data);

 private:
  using ScopedPipe::ScopedPipe;

  friend class ScopedPipe;
};

}  //  namespace aos::util

#endif  // AOS_UTIL_SCOPED_PIPE_H_
