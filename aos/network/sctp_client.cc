#include "aos/network/sctp_client.h"

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/sctp.h>
#include <sys/socket.h>

#include <cstdlib>
#include <cstring>
#include <string_view>

#include "aos/network/sctp_lib.h"
#include "aos/unique_malloc_ptr.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {

SctpClient::SctpClient(std::string_view remote_host, int remote_port,
                       int streams, std::string_view local_host,
                       int local_port) {
  bool use_ipv6 = Ipv6Enabled();
  sockaddr_local_ = ResolveSocket(local_host, local_port, use_ipv6);
  sockaddr_remote_ = ResolveSocket(remote_host, remote_port, use_ipv6);
  sctp_.OpenSocket(sockaddr_local_);

  {
    struct sctp_initmsg initmsg;
    memset(&initmsg, 0, sizeof(struct sctp_initmsg));
    initmsg.sinit_num_ostreams = streams;
    initmsg.sinit_max_instreams = streams;
    PCHECK(setsockopt(fd(), IPPROTO_SCTP, SCTP_INITMSG, &initmsg,
                      sizeof(struct sctp_initmsg)) == 0);
  }

  {
    // Turn off the NAGLE algorithm so the timestamps heading back across the
    // network arrive promptly.
    int on = 1;
    PCHECK(setsockopt(fd(), IPPROTO_SCTP, SCTP_NODELAY, &on, sizeof(int)) == 0);
  }
}

void SctpClient::LogSctpStatus(sctp_assoc_t assoc_id) {
  message_bridge::LogSctpStatus(fd(), assoc_id);
}

void SctpClient::SetPriorityScheduler(sctp_assoc_t assoc_id) {
  struct sctp_assoc_value scheduler;
  memset(&scheduler, 0, sizeof(scheduler));
  scheduler.assoc_id = assoc_id;
  scheduler.assoc_value = SCTP_SS_PRIO;
  if (setsockopt(fd(), IPPROTO_SCTP, SCTP_STREAM_SCHEDULER, &scheduler,
                 sizeof(scheduler)) != 0) {
    LOG_FIRST_N(WARNING, 1) << "Failed to set scheduler: " << strerror(errno)
                            << " [" << errno << "]";
  }
}

}  // namespace message_bridge
}  // namespace aos
