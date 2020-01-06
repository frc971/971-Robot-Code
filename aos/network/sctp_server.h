#ifndef AOS_NETWORK_SCTP_SERVER_H_
#define AOS_NETWORK_SCTP_SERVER_H_

#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/sctp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>
#include <sys/socket.h>

#include "aos/network/sctp_lib.h"
#include "aos/unique_malloc_ptr.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {

class SctpServer {
 public:
  SctpServer(std::string_view local_host = "0.0.0.0", int local_port = 9971);

  ~SctpServer() {
    LOG(INFO) << "close(" << fd_ << ")";
    PCHECK(close(fd_) == 0);
  }

  // Receives the next packet from the remote.
  aos::unique_c_ptr<Message> Read();

  // Sends a block of data to a client on a stream with a TTL.
  void Send(std::string_view data, sctp_assoc_t snd_assoc_id, int stream,
            int timetolive);

  int fd() { return fd_; }

  // Enables the priority scheduler.  This is a SCTP feature which lets us
  // configure the priority per stream so that higher priority packets don't get
  // backed up behind lower priority packets in the networking queues.
  void SetPriorityScheduler(sctp_assoc_t assoc_id);

  // Sets the priority of a specific stream.
  void SetStreamPriority(sctp_assoc_t assoc_id, int stream_id,
                         uint16_t priority);

 private:
  struct sockaddr_storage sockaddr_local_;
  int fd_;

  // TODO(austin): Configure this.
  size_t max_size_ = 1000;

  int ppid_ = 1;
};


}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_SCTP_SERVER_H_
