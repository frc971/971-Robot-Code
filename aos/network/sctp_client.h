#ifndef AOS_NETWORK_SCTP_CLIENT_H_
#define AOS_NETWORK_SCTP_CLIENT_H_

#include <cstdio>
#include <cstdlib>
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

  ~SctpClient() {}

  // Receives the next packet from the remote.
  aos::unique_c_ptr<Message> Read() { return sctp_.ReadMessage(); }

  // Sends a block of data on a stream with a TTL.
  // TODO(austin): time_to_live should be a chrono::duration
  bool Send(int stream, std::string_view data, int time_to_live) {
    return sctp_.SendMessage(stream, data, time_to_live, sockaddr_remote_, 0);
  }

  int fd() { return sctp_.fd(); }

  // Enables the priority scheduler.  This is a SCTP feature which lets us
  // configure the priority per stream so that higher priority packets don't get
  // backed up behind lower priority packets in the networking queues.
  void SetPriorityScheduler(sctp_assoc_t assoc_id);

  // Remote to send to.
  struct sockaddr_storage sockaddr_remote() const {
    return sockaddr_remote_;
  }

  void LogSctpStatus(sctp_assoc_t assoc_id);

  void SetMaxSize(size_t max_size) { sctp_.SetMaxSize(max_size); }

 private:
  struct sockaddr_storage sockaddr_remote_;
  struct sockaddr_storage sockaddr_local_;
  SctpReadWrite sctp_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  //  AOS_NETWORK_SCTP_CLIENT_H_
