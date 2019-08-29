#include "aos/scoped/scoped_fd.h"

#include "glog/logging.h"

namespace aos {

void ScopedFD::Close() {
  if (fd_ != -1) {
    if (close(fd_) == -1) {
      PLOG(WARNING) << "close(" << fd_ << ") failed";
    }
  }
}

}  // namespace aos
