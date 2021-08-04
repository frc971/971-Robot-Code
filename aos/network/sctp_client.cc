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
                       int streams, std::string_view local_host, int local_port)
    : sockaddr_remote_(ResolveSocket(remote_host, remote_port)),
      sockaddr_local_(ResolveSocket(local_host, local_port)) {
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
    // Servers send promptly.  Clients don't.
    // TODO(austin): Revisit this assumption when we have time sync.
    int on = 0;
    PCHECK(setsockopt(fd(), IPPROTO_SCTP, SCTP_NODELAY, &on, sizeof(int)) == 0);
  }

  {
    // TODO(austin): This is the old style registration...  But, the sctp
    // stack out in the wild for linux is old and primitive.
    struct sctp_event_subscribe subscribe;
    memset(&subscribe, 0, sizeof(subscribe));
    subscribe.sctp_association_event = 1;
    subscribe.sctp_stream_change_event = 1;
    PCHECK(setsockopt(fd(), SOL_SCTP, SCTP_EVENTS, (char *)&subscribe,
                      sizeof(subscribe)) == 0);
  }

  PCHECK(bind(fd(), (struct sockaddr *)&sockaddr_local_,
              sockaddr_local_.ss_family == AF_INET6
                  ? sizeof(struct sockaddr_in6)
                  : sizeof(struct sockaddr_in)) == 0);
  VLOG(1) << "bind(" << fd() << ", " << Address(sockaddr_local_) << ")";
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
    PLOG(WARNING) << "Failed to set scheduler";
  }
}

}  // namespace message_bridge
}  // namespace aos
