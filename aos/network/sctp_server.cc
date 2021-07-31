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
    fd_ = socket(sockaddr_local_.ss_family, SOCK_SEQPACKET, IPPROTO_SCTP);
    LOG(INFO) << "socket(" << Family(sockaddr_local_)
              << ", SOCK_SEQPACKET, IPPROTOSCTP) = " << fd_;
    PCHECK(fd_ != -1);

    {
      struct sctp_event_subscribe subscribe;
      memset(&subscribe, 0, sizeof(subscribe));
      subscribe.sctp_association_event = 1;
      subscribe.sctp_send_failure_event = 1;
      subscribe.sctp_partial_delivery_event = 1;

      PCHECK(setsockopt(fd_, SOL_SCTP, SCTP_EVENTS, (char *)&subscribe,
                        sizeof(subscribe)) == 0);
    }
    {
      // Enable recvinfo when a packet arrives.
      int on = 1;
      PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_RECVRCVINFO, &on,
                        sizeof(int)) == 0);
    }
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
      // Turn off the NAGLE algorithm.
      int on = 1;
      PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_NODELAY, &on, sizeof(int)) ==
             0);
    }

    {
      int on = 1;
      PCHECK(setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)) == 0);
    }

    // And go!
    if (bind(fd_, (struct sockaddr *)&sockaddr_local_,
             sockaddr_local_.ss_family == AF_INET6
                 ? sizeof(struct sockaddr_in6)
                 : sizeof(struct sockaddr_in)) != 0) {
      PLOG(ERROR) << "Failed to bind, retrying";
      close(fd_);
      std::this_thread::sleep_for(std::chrono::seconds(5));
      continue;
    }
    LOG(INFO) << "bind(" << fd_ << ", " << Address(sockaddr_local_) << ")";

    PCHECK(listen(fd_, 100) == 0);

    SetMaxSize(1000);
    break;
  }
}

aos::unique_c_ptr<Message> SctpServer::Read() {
  return ReadSctpMessage(fd_, max_size_);
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
  sinfo->sinfo_ppid = ++ppid_;
  sinfo->sinfo_stream = 0;
  sinfo->sinfo_flags = SCTP_ABORT;
  sinfo->sinfo_assoc_id = snd_assoc_id;

  // And send.
  const ssize_t size = sendmsg(fd_, &outmsg, MSG_NOSIGNAL | MSG_DONTWAIT);
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

bool SctpServer::Send(std::string_view data, sctp_assoc_t snd_assoc_id,
                      int stream, int timetolive) {
  struct iovec iov;
  iov.iov_base = const_cast<char *>(data.data());
  iov.iov_len = data.size();

  // Use the assoc_id for the destination instead of the msg_name.
  struct msghdr outmsg;
  outmsg.msg_namelen = 0;

  // Data to send.
  outmsg.msg_iov = &iov;
  outmsg.msg_iovlen = 1;

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
  sinfo->sinfo_ppid = ++ppid_;
  sinfo->sinfo_stream = stream;
  sinfo->sinfo_flags = 0;
  sinfo->sinfo_assoc_id = snd_assoc_id;
  sinfo->sinfo_timetolive = timetolive;

  // And send.
  const ssize_t size = sendmsg(fd_, &outmsg, MSG_NOSIGNAL | MSG_DONTWAIT);
  if (size == -1) {
    if (errno == EPIPE || errno == EAGAIN || errno == ESHUTDOWN) {
      return false;
    }
    PCHECK(size == static_cast<ssize_t>(data.size()));
    return false;
  } else {
    CHECK_EQ(static_cast<ssize_t>(data.size()), size);
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
