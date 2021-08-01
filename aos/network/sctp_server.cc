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

SctpServer::SctpServer(std::string_view local_host, int local_port)
    : sockaddr_local_(ResolveSocket(local_host, local_port)) {
  while (true) {
    sctp_.OpenSocket(sockaddr_local_);

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

bool SctpServer::Abort(sctp_assoc_t snd_assoc_id) {
  // Use the assoc_id for the destination instead of the msg_name.
  struct msghdr outmsg;
  outmsg.msg_namelen = 0;

  outmsg.msg_iovlen = 0;

  // Build up the sndinfo message.
  char outcmsg[CMSG_SPACE(sizeof(struct sctp_sndrcvinfo))];
  outmsg.msg_control = outcmsg;
  outmsg.msg_controllen = CMSG_SPACE(sizeof(struct sctp_sndrcvinfo));
  outmsg.msg_flags = 0;

  struct cmsghdr *cmsg = CMSG_FIRSTHDR(&outmsg);
  cmsg->cmsg_level = IPPROTO_SCTP;
  cmsg->cmsg_type = SCTP_SNDRCV;
  cmsg->cmsg_len = CMSG_LEN(sizeof(struct sctp_sndrcvinfo));

  struct sctp_sndrcvinfo *sinfo = (struct sctp_sndrcvinfo *)CMSG_DATA(cmsg);
  memset(sinfo, 0, sizeof(struct sctp_sndrcvinfo));
  sinfo->sinfo_stream = 0;
  sinfo->sinfo_flags = SCTP_ABORT;
  sinfo->sinfo_assoc_id = snd_assoc_id;

  // And send.
  const ssize_t size = sendmsg(fd(), &outmsg, MSG_NOSIGNAL | MSG_DONTWAIT);
  if (size == -1) {
    if (errno == EPIPE || errno == EAGAIN || errno == ESHUTDOWN) {
      return false;
    }
    return false;
  } else {
    CHECK_EQ(0, size);
    return true;
  }
}

void SctpServer::SetPriorityScheduler(sctp_assoc_t assoc_id) {
  struct sctp_assoc_value scheduler;
  memset(&scheduler, 0, sizeof(scheduler));
  scheduler.assoc_id = assoc_id;
  scheduler.assoc_value = SCTP_SS_PRIO;
  if (setsockopt(fd(), IPPROTO_SCTP, SCTP_STREAM_SCHEDULER, &scheduler,
                 sizeof(scheduler)) != 0) {
    PLOG(WARNING) << "Failed to set scheduler";
  }
}

void SctpServer::SetStreamPriority(sctp_assoc_t assoc_id, int stream_id,
                                   uint16_t priority) {
  struct sctp_stream_value sctp_priority;
  memset(&sctp_priority, 0, sizeof(sctp_priority));
  sctp_priority.assoc_id = assoc_id;
  sctp_priority.stream_id = stream_id;
  sctp_priority.stream_value = priority;
  if (setsockopt(fd(), IPPROTO_SCTP, SCTP_STREAM_SCHEDULER_VALUE,
                 &sctp_priority, sizeof(sctp_priority)) != 0) {
    PLOG(WARNING) << "Failed to set scheduler";
  }
}

}  // namespace message_bridge
}  // namespace aos
