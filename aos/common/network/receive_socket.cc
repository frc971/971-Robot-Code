#include "aos/common/network/receive_socket.h"

#include <string.h>
#include <math.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/socket.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace network {

static const char *localhost = "0.0.0.0";

int ReceiveSocket::Connect(NetworkPort port) {
  Reset();
  const int ret = Socket::Connect(port, localhost);
  if (ret != 0) {
    return ret;
  }

  if (bind(socket_, &addr_.addr,
           sizeof(addr_)) == -1) {
    PLOG(ERROR, "failed to bind to address '%s'", localhost);
    return last_ret_ = -1;
  }
  return last_ret_ = 0;
}

}  // namespace network
}  // namespace aos
