#include <unistd.h>

#include "aos/common/macros.h"
#include "aos/common/logging/logging.h"

namespace aos {

// Smart "pointer" (container) for a file descriptor.
class ScopedFD {
 public:
  explicit ScopedFD(int fd = -1) : fd_(fd) {}
  ~ScopedFD() {
    Close();
  }
  int get() const { return fd_; }
  int release() {
    const int r = fd_;
    fd_ = -1;
    return r;
  }
  void reset(int new_fd = -1) {
    if (fd_ != new_fd) {
      Close();
      fd_ = new_fd;
    }
  }
  operator bool() const { return fd_ != -1; }
 private:
  int fd_;
  void Close() {
    if (fd_ != -1) {
      if (close(fd_) == -1) {
        PLOG(WARNING, "close(%d) failed", fd_);
      }
    }
  }
  DISALLOW_COPY_AND_ASSIGN(ScopedFD);
};

}  // namespace aos
