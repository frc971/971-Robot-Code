#include "aos/network/sctp_server.h"

#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/sctp.h>
#include <sys/socket.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

#include "aos/network/sctp_lib.h"
#include "aos/unique_malloc_ptr.h"
#include "glog/logging.h"

namespace aos {
namespace message_bridge {

SctpServer::SctpServer(int streams, std::string_view local_host,
                       int local_port) {
  bool use_ipv6 = Ipv6Enabled();
  sockaddr_local_ = ResolveSocket(local_host, local_port, use_ipv6);
  while (true) {
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
      // Turn off the NAGLE algorithm.
      int on = 1;
      PCHECK(setsockopt(fd(), IPPROTO_SCTP, SCTP_NODELAY, &on, sizeof(int)) ==
             0);
    }

    {
      int on = 1;
      LOG(INFO) << "setsockopt(" << fd()
                << ", SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)";
      PCHECK(setsockopt(fd(), SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)) == 0);
    }

    // And go!
    if (bind(fd(), (struct sockaddr *)&sockaddr_local_,
             sockaddr_local_.ss_family == AF_INET6
                 ? sizeof(struct sockaddr_in6)
                 : sizeof(struct sockaddr_in)) != 0) {
      PLOG(ERROR) << "Failed to bind, retrying";
      close(fd());
      std::this_thread::sleep_for(std::chrono::seconds(5));
      continue;
    }
    LOG(INFO) << "bind(" << fd() << ", " << Address(sockaddr_local_) << ")";

    PCHECK(listen(fd(), 100) == 0);

    SetMaxSize(1000);
    break;
  }
}

void SctpServer::SetPriorityScheduler([[maybe_unused]] sctp_assoc_t assoc_id) {
// Kernel 4.9 does not have SCTP_SS_PRIO
#ifdef SCTP_SS_PRIO
  struct sctp_assoc_value scheduler;
  memset(&scheduler, 0, sizeof(scheduler));
  scheduler.assoc_id = assoc_id;
  scheduler.assoc_value = SCTP_SS_PRIO;
  if (setsockopt(fd(), IPPROTO_SCTP, SCTP_STREAM_SCHEDULER, &scheduler,
                 sizeof(scheduler)) != 0) {
    LOG_FIRST_N(WARNING, 1) << "Failed to set scheduler: " << strerror(errno)
                            << " [" << errno << "]";
  }
#endif
}

void SctpServer::SetStreamPriority([[maybe_unused]] sctp_assoc_t assoc_id,
                                   [[maybe_unused]] int stream_id,
                                   [[maybe_unused]] uint16_t priority) {
// Kernel 4.9 does not have SCTP_STREAM_SCHEDULER_VALUE
#ifdef SCTP_STREAM_SCHEDULER_VALUE
  struct sctp_stream_value sctp_priority;
  memset(&sctp_priority, 0, sizeof(sctp_priority));
  sctp_priority.assoc_id = assoc_id;
  sctp_priority.stream_id = stream_id;
  sctp_priority.stream_value = priority;
  if (setsockopt(fd(), IPPROTO_SCTP, SCTP_STREAM_SCHEDULER_VALUE,
                 &sctp_priority, sizeof(sctp_priority)) != 0) {
    LOG_FIRST_N(WARNING, 1) << "Failed to set scheduler: " << strerror(errno)
                            << " [" << errno << "]";
  }
#endif
}

}  // namespace message_bridge
}  // namespace aos
