#include "aos/common/network/SendSocket.h"

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "aos/aos_core.h"
#include "aos/common/network/SocketLibraries.h"

namespace aos {

int SendSocket::Connect(NetworkPort port, const char *robot_ip, int type) {
  Reset();
  const int ret = Socket::Connect(port, robot_ip, type);
  if (ret != 0) {
    return ret;
  }

  if (connect(socket_, &addr_.addr,
              sizeof(addr_)) < 0) {
    LOG(ERROR, "couldn't connect to ip '%s' because of %d: %s\n", robot_ip,
        errno, strerror(errno));
    last_ret_ = 1;
  }

  hold_msg_len_ = 0;
  return last_ret_ = 0;
}

int SendSocket::SendHoldMsg() {
  if (hold_msg_len_ > 0) {
    int val = Send(hold_msg_, hold_msg_len_);
    hold_msg_len_ = 0;
    return val;
  }
  return -1;
}

} // namespace aos

