#ifndef AOS_NETWORK_SCTP_CLIENT_H_
#define AOS_NETWORK_SCTP_CLIENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string_view>

#include "aos/network/sctp_lib.h"
#include "aos/unique_malloc_ptr.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {

// Class to encapsulate everything needed to be a SCTP client.
class SctpClient {
 public:
  SctpClient(std::string_view remote_host, int remote_port, int streams,
             std::string_view local_host = "0.0.0.0", int local_port = 9971);

  ~SctpClient() {
    LOG(INFO) << "close(" << fd_ << ")";
    PCHECK(close(fd_) == 0);
  }

  // Receives the next packet from the remote.
  aos::unique_c_ptr<Message> Read();

  // Sends a block of data on a stream with a TTL.
  bool Send(int stream, std::string_view data, int time_to_live);

  int fd() { return fd_; }

  // Enables the priority scheduler.  This is a SCTP feature which lets us
  // configure the priority per stream so that higher priority packets don't get
  // backed up behind lower priority packets in the networking queues.
  void SetPriorityScheduler(sctp_assoc_t assoc_id);

  // Remote to send to.
  struct sockaddr_storage sockaddr_remote() const {
    return sockaddr_remote_;
  }

  void LogSctpStatus(sctp_assoc_t assoc_id);

 private:
  struct sockaddr_storage sockaddr_remote_;
  struct sockaddr_storage sockaddr_local_;
  int fd_;

  size_t max_size_ = 1000;
};

}  // namespace message_bridge
}  // namespace aos

#endif  //  AOS_NETWORK_SCTP_CLIENT_H_
