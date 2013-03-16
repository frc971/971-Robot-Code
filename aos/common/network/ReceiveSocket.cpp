#include "ReceiveSocket.h"
#include <string.h>
#include <math.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>

#include "aos/common/network/SocketLibraries.h"
#include "aos/aos_core.h"

namespace aos {

static const char *localhost = "0.0.0.0";

int ReceiveSocket::Connect(NetworkPort port) {
  Reset();
  const int ret = Socket::Connect(port, localhost);
  if (ret != 0) {
    return ret;
  }

  if (bind(socket_, &addr_.addr,
           sizeof(addr_)) == -1) {
    LOG(ERROR, "failed to bind to address '%s' because of %d: %s\n", localhost,
        errno, strerror(errno));
    return last_ret_ = -1;
  }
  return last_ret_ = 0;
}

} // namespace aos

