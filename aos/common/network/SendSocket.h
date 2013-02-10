#ifndef AOS_NETWORK_SEND_SOCKET_H_
#define AOS_NETWORK_SEND_SOCKET_H_

#include "Socket.h"

namespace aos {

class SendSocket : public Socket {
 public:
	//inline int Send(const void *buf, int length) { return Socket::Send(buf, length); }
  // Connect must be called before use.
  SendSocket() {}
  // Calls Connect automatically.
	SendSocket(NetworkPort port, const char *robot_ip) {
    Connect(port, robot_ip);
  }
	int Connect(NetworkPort port, const char *robot_ip, int type = SOCK_DGRAM);

  static const size_t MAX_MSG = 4096;
	char hold_msg_[MAX_MSG];
	size_t hold_msg_len_;
	int SendHoldMsg();
};

} // namespace aos

#endif

