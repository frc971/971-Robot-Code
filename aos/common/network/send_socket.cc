#include "aos/common/network/send_socket.h"

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <sys/socket.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace network {

int SendSocket::Connect(NetworkPort port, const char *robot_ip, int type) {
  Reset();
  const int ret = Socket::Connect(port, robot_ip, type);
  if (ret != 0) {
    return ret;
  }

  if (connect(socket_, &addr_.addr, sizeof(addr_)) < 0) {
    PLOG(ERROR, "couldn't connect to ip '%s'", robot_ip);
    return last_ret_ = 1;
  }

  return last_ret_ = 0;
}

}  // namespace network
}  // namespace aos
