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
      sockaddr_local_(ResolveSocket(local_host, local_port)),
      fd_(socket(sockaddr_local_.ss_family, SOCK_SEQPACKET, IPPROTO_SCTP)) {
  LOG(INFO) << "socket(" << Family(sockaddr_local_)
            << ", SOCK_SEQPACKET, IPPROTOSCTP) = " << fd_;
  PCHECK(fd_ != -1);

  {
    // Per https://tools.ietf.org/html/rfc6458
    // Setting this to !0 allows event notifications to be interleaved
    // with data if enabled, and would have to be handled in the code.
    // Enabling interleaving would only matter during congestion, which
    // typically only happens during application startup.
    int interleaving = 0;
    PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_FRAGMENT_INTERLEAVE,
                      &interleaving, sizeof(interleaving)) == 0);
  }

  {
    struct sctp_initmsg initmsg;
    memset(&initmsg, 0, sizeof(struct sctp_initmsg));
    initmsg.sinit_num_ostreams = streams;
    initmsg.sinit_max_instreams = streams;
    PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_INITMSG, &initmsg,
                      sizeof(struct sctp_initmsg)) == 0);
  }

  {
    int on = 1;
    PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_RECVRCVINFO, &on, sizeof(int)) ==
           0);
  }
  {
    // Servers send promptly.  Clients don't.
    // TODO(austin): Revisit this assumption when we have time sync.
    int on = 0;
    PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_NODELAY, &on, sizeof(int)) == 0);
  }

  {
    // TODO(austin): This is the old style registration...  But, the sctp
    // stack out in the wild for linux is old and primitive.
    struct sctp_event_subscribe subscribe;
    memset(&subscribe, 0, sizeof(subscribe));
    subscribe.sctp_association_event = 1;
    subscribe.sctp_stream_change_event = 1;
    PCHECK(setsockopt(fd_, SOL_SCTP, SCTP_EVENTS, (char *)&subscribe,
                      sizeof(subscribe)) == 0);
  }

  PCHECK(bind(fd_, (struct sockaddr *)&sockaddr_local_,
              sockaddr_local_.ss_family == AF_INET6
                  ? sizeof(struct sockaddr_in6)
                  : sizeof(struct sockaddr_in)) == 0);
  VLOG(1) << "bind(" << fd_ << ", " << Address(sockaddr_local_) << ")";
}

aos::unique_c_ptr<Message> SctpClient::Read() {
  return ReadSctpMessage(fd_, max_size_);
}

bool SctpClient::Send(int stream, std::string_view data, int time_to_live) {
  struct iovec iov;
  iov.iov_base = const_cast<char *>(data.data());
  iov.iov_len = data.size();

  struct msghdr outmsg;
  // Target to send to.
  outmsg.msg_name = &sockaddr_remote_;
  outmsg.msg_namelen = sizeof(struct sockaddr_storage);
  VLOG(1) << "Sending to " << Address(sockaddr_remote_);

  // Data to send.
  outmsg.msg_iov = &iov;
  outmsg.msg_iovlen = 1;

  // Build up the sndinfo message.
  char outcmsg[CMSG_SPACE(sizeof(struct sctp_sndrcvinfo))];
  outmsg.msg_control = outcmsg;
  outmsg.msg_controllen = sizeof(outcmsg);
  outmsg.msg_flags = 0;

  struct cmsghdr *cmsg = CMSG_FIRSTHDR(&outmsg);
  cmsg->cmsg_level = IPPROTO_SCTP;
  cmsg->cmsg_type = SCTP_SNDRCV;
  cmsg->cmsg_len = CMSG_LEN(sizeof(struct sctp_sndrcvinfo));

  outmsg.msg_controllen = cmsg->cmsg_len;
  struct sctp_sndrcvinfo *sinfo = (struct sctp_sndrcvinfo *)CMSG_DATA(cmsg);
  memset(sinfo, 0, sizeof(struct sctp_sndrcvinfo));
  sinfo->sinfo_ppid = rand();
  sinfo->sinfo_stream = stream;
  sinfo->sinfo_context = 19;
  sinfo->sinfo_flags = 0;
  sinfo->sinfo_timetolive = time_to_live;

  // And send.
  const ssize_t size = sendmsg(fd_, &outmsg, MSG_NOSIGNAL | MSG_DONTWAIT);
  if (size == -1) {
    if (errno != EPIPE && errno != EAGAIN && errno != ESHUTDOWN) {
      PCHECK(size == static_cast<ssize_t>(data.size()));
    } else {
      return false;
    }
  } else {
    CHECK_EQ(static_cast<ssize_t>(data.size()), size);
  }

  VLOG(1) << "Sent " << data.size();
  return true;
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
