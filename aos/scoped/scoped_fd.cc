#include "aos/scoped/scoped_fd.h"

#include "aos/logging/logging.h"

namespace aos {

void ScopedFD::Close() {
  if (fd_ != -1) {
    if (close(fd_) == -1) {
      PLOG(WARNING, "close(%d) failed", fd_);
    }
  }
}

}  // namespace aos
