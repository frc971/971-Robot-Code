#include "aos/network/sctp_lib.h"

#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/sctp.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <string_view>

#include "aos/util/file.h"

DEFINE_string(interface, "", "ipv6 interface");

namespace aos {
namespace message_bridge {

namespace {
const char *sac_state_tbl[] = {"COMMUNICATION_UP", "COMMUNICATION_LOST",
                               "RESTART", "SHUTDOWN_COMPLETE",
                               "CANT_START_ASSOCICATION"};

typedef union {
  struct sctp_initmsg init;
  struct sctp_sndrcvinfo sndrcvinfo;
} _sctp_cmsg_data_t;

}  // namespace

struct sockaddr_storage ResolveSocket(std::string_view host, int port) {
  struct sockaddr_storage result;
  struct addrinfo *addrinfo_result;
  struct sockaddr_in *t_addr = (struct sockaddr_in *)&result;
  struct sockaddr_in6 *t_addr6 = (struct sockaddr_in6 *)&result;

  PCHECK(getaddrinfo(std::string(host).c_str(), 0, NULL, &addrinfo_result) == 0)
      << ": Failed to look up " << host;

  switch (addrinfo_result->ai_family) {
    case AF_INET:
      memcpy(t_addr, addrinfo_result->ai_addr, addrinfo_result->ai_addrlen);
      t_addr->sin_family = addrinfo_result->ai_family;
      t_addr->sin_port = htons(port);

      break;
    case AF_INET6:
      memcpy(t_addr6, addrinfo_result->ai_addr, addrinfo_result->ai_addrlen);
      t_addr6->sin6_family = addrinfo_result->ai_family;
      t_addr6->sin6_port = htons(port);

      if (FLAGS_interface.size() > 0) {
        t_addr6->sin6_scope_id = if_nametoindex(FLAGS_interface.c_str());
      }

      break;
  }

  // Now print it back out nicely.
  char host_string[NI_MAXHOST];
  char service_string[NI_MAXSERV];

  int error = getnameinfo((struct sockaddr *)&result,
                          addrinfo_result->ai_addrlen, host_string, NI_MAXHOST,
                          service_string, NI_MAXSERV, NI_NUMERICHOST);

  if (error) {
    LOG(ERROR) << "Reverse lookup failed ... " << gai_strerror(error);
  }

  LOG(INFO) << "remote:addr=" << host_string << ", port=" << service_string
            << ", family=" << addrinfo_result->ai_family;

  freeaddrinfo(addrinfo_result);

  return result;
}

std::string_view Family(const struct sockaddr_storage &sockaddr) {
  if (sockaddr.ss_family == AF_INET) {
    return "AF_INET";
  } else if (sockaddr.ss_family == AF_INET6) {
    return "AF_INET6";
  } else {
    return "unknown";
  }
}
std::string Address(const struct sockaddr_storage &sockaddr) {
  char addrbuf[INET6_ADDRSTRLEN];
  if (sockaddr.ss_family == AF_INET) {
    const struct sockaddr_in *sin = (const struct sockaddr_in *)&sockaddr;
    return std::string(
        inet_ntop(AF_INET, &sin->sin_addr, addrbuf, INET6_ADDRSTRLEN));
  } else {
    const struct sockaddr_in6 *sin6 = (const struct sockaddr_in6 *)&sockaddr;
    return std::string(
        inet_ntop(AF_INET6, &sin6->sin6_addr, addrbuf, INET6_ADDRSTRLEN));
  }
}

void PrintNotification(const Message *msg) {
  const union sctp_notification *snp =
      (const union sctp_notification *)msg->data();

  LOG(INFO) << "Notification:";

  switch (snp->sn_header.sn_type) {
    case SCTP_ASSOC_CHANGE: {
      const struct sctp_assoc_change *sac = &snp->sn_assoc_change;
      LOG(INFO) << "SCTP_ASSOC_CHANGE(" << sac_state_tbl[sac->sac_state] << ")";
      VLOG(1) << "    (assoc_change: state=" << sac->sac_state
              << ", error=" << sac->sac_error
              << ", instr=" << sac->sac_inbound_streams
              << " outstr=" << sac->sac_outbound_streams
              << ", assoc=" << sac->sac_assoc_id << ")";
    } break;
    case SCTP_PEER_ADDR_CHANGE: {
      const struct sctp_paddr_change *spc = &snp->sn_paddr_change;
      LOG(INFO) << " SlCTP_PEER_ADDR_CHANGE";
      VLOG(1) << "\t\t(peer_addr_change: " << Address(spc->spc_aaddr)
              << " state=" << spc->spc_state << ", error=" << spc->spc_error
              << ")";
    } break;
    case SCTP_SEND_FAILED: {
      const struct sctp_send_failed *ssf = &snp->sn_send_failed;
      LOG(INFO) << " SCTP_SEND_FAILED";
      VLOG(1) << "\t\t(sendfailed: len=" << ssf->ssf_length
              << " err=" << ssf->ssf_error << ")";
    } break;
    case SCTP_REMOTE_ERROR: {
      const struct sctp_remote_error *sre = &snp->sn_remote_error;
      LOG(INFO) << " SCTP_REMOTE_ERROR";
      VLOG(1) << "\t\t(remote_error: err=" << ntohs(sre->sre_error) << ")";
    } break;
    case SCTP_STREAM_CHANGE_EVENT: {
      const struct sctp_stream_change_event *sce = &snp->sn_strchange_event;
      LOG(INFO) << " SCTP_STREAM_CHANGE_EVENT";
      VLOG(1) << "\t\t(stream_change_event: flags=" << sce->strchange_flags
              << ", assoc_id=" << sce->strchange_assoc_id
              << ", instrms=" << sce->strchange_instrms
              << ", outstrms=" << sce->strchange_outstrms << " )";
    } break;
    case SCTP_SHUTDOWN_EVENT: {
      LOG(INFO) << " SCTP_SHUTDOWN_EVENT";
    } break;
    default:
      LOG(INFO) << " Unknown type: " << snp->sn_header.sn_type;
      break;
  }
}

std::string GetHostname() {
  char buf[256];
  buf[sizeof(buf) - 1] = '\0';
  PCHECK(gethostname(buf, sizeof(buf) - 1) == 0);
  return buf;
}

std::string Message::PeerAddress() const { return Address(sin); }

void LogSctpStatus(int fd, sctp_assoc_t assoc_id) {
  struct sctp_status status;
  memset(&status, 0, sizeof(status));
  status.sstat_assoc_id = assoc_id;

  socklen_t size = sizeof(status);
  const int result = getsockopt(fd, SOL_SCTP, SCTP_STATUS,
                                reinterpret_cast<void *>(&status), &size);
  if (result == -1 && errno == EINVAL) {
    LOG(INFO) << "sctp_status) not associated";
    return;
  }
  PCHECK(result == 0);

  LOG(INFO) << "sctp_status) sstat_assoc_id:" << status.sstat_assoc_id
            << " sstat_state:" << status.sstat_state
            << " sstat_rwnd:" << status.sstat_rwnd
            << " sstat_unackdata:" << status.sstat_unackdata
            << " sstat_penddata:" << status.sstat_penddata
            << " sstat_instrms:" << status.sstat_instrms
            << " sstat_outstrms:" << status.sstat_outstrms
            << " sstat_fragmentation_point:" << status.sstat_fragmentation_point
            << " sstat_primary.spinfo_srtt:" << status.sstat_primary.spinfo_srtt
            << " sstat_primary.spinfo_rto:" << status.sstat_primary.spinfo_rto;
}

void SctpReadWrite::OpenSocket(const struct sockaddr_storage &sockaddr_local) {
  fd_ = socket(sockaddr_local.ss_family, SOCK_SEQPACKET, IPPROTO_SCTP);
  PCHECK(fd_ != -1);
  LOG(INFO) << "socket(" << Family(sockaddr_local)
            << ", SOCK_SEQPACKET, IPPROTOSCTP) = " << fd_;
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
    // Enable recvinfo when a packet arrives.
    int on = 1;
    PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_RECVRCVINFO, &on, sizeof(int)) ==
           0);
  }

  DoSetMaxSize();
}

bool SctpReadWrite::SendMessage(
    int stream, std::string_view data, int time_to_live,
    std::optional<struct sockaddr_storage> sockaddr_remote,
    sctp_assoc_t snd_assoc_id) {
  CHECK(fd_ != -1);
  struct iovec iov;
  iov.iov_base = const_cast<char *>(data.data());
  iov.iov_len = data.size();

  // Use the assoc_id for the destination instead of the msg_name.
  struct msghdr outmsg;
  if (sockaddr_remote) {
    outmsg.msg_name = &*sockaddr_remote;
    outmsg.msg_namelen = sizeof(*sockaddr_remote);
    VLOG(1) << "Sending to " << Address(*sockaddr_remote);
  } else {
    outmsg.msg_namelen = 0;
  }

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

  struct sctp_sndrcvinfo *sinfo =
      reinterpret_cast<struct sctp_sndrcvinfo *>(CMSG_DATA(cmsg));
  memset(sinfo, 0, sizeof(struct sctp_sndrcvinfo));
  sinfo->sinfo_ppid = ++send_ppid_;
  sinfo->sinfo_stream = stream;
  sinfo->sinfo_flags = 0;
  sinfo->sinfo_assoc_id = snd_assoc_id;
  sinfo->sinfo_timetolive = time_to_live;

  // And send.
  const ssize_t size = sendmsg(fd_, &outmsg, MSG_NOSIGNAL | MSG_DONTWAIT);
  if (size == -1) {
    if (errno == EPIPE || errno == EAGAIN || errno == ESHUTDOWN ||
        errno == EINTR) {
      return false;
    }
    PLOG(FATAL) << "sendmsg on sctp socket failed";
    return false;
  }
  CHECK_EQ(static_cast<ssize_t>(data.size()), size);
  VLOG(1) << "Sent " << data.size();
  return true;
}

aos::unique_c_ptr<Message> SctpReadWrite::ReadMessage() {
  CHECK(fd_ != -1);
  aos::unique_c_ptr<Message> result(
      reinterpret_cast<Message *>(malloc(sizeof(Message) + max_size_ + 1)));
  result->size = 0;

  int count = 0;
  int last_flags = 0;
  for (count = 0; !(last_flags & MSG_EOR); count++) {
    struct msghdr inmessage;
    memset(&inmessage, 0, sizeof(struct msghdr));

    struct iovec iov;
    iov.iov_len = max_size_ + 1 - result->size;
    iov.iov_base = result->mutable_data() + result->size;

    inmessage.msg_iov = &iov;
    inmessage.msg_iovlen = 1;

    char incmsg[CMSG_SPACE(sizeof(_sctp_cmsg_data_t))];
    inmessage.msg_control = incmsg;
    inmessage.msg_controllen = sizeof(incmsg);

    inmessage.msg_namelen = sizeof(struct sockaddr_storage);
    inmessage.msg_name = &result->sin;

    ssize_t size;
    PCHECK((size = recvmsg(fd_, &inmessage, 0)) > 0);

    if (count > 0) {
      VLOG(1) << "Count: " << count;
      VLOG(1) << "Last msg_flags: " << last_flags;
      VLOG(1) << "msg_flags: " << inmessage.msg_flags;
      VLOG(1) << "Current size: " << result->size;
      VLOG(1) << "Received size: " << size;
      CHECK_EQ(MSG_NOTIFICATION & inmessage.msg_flags,
               MSG_NOTIFICATION & last_flags);
    }

    result->size += size;
    last_flags = inmessage.msg_flags;

    for (struct cmsghdr *scmsg = CMSG_FIRSTHDR(&inmessage); scmsg != NULL;
         scmsg = CMSG_NXTHDR(&inmessage, scmsg)) {
      switch (scmsg->cmsg_type) {
        case SCTP_RCVINFO: {
          struct sctp_rcvinfo *data =
              reinterpret_cast<struct sctp_rcvinfo *>(CMSG_DATA(scmsg));
          if (count > 0) {
            VLOG(1) << "Got sctp_rcvinfo on continued packet";
            CHECK_EQ(result->header.rcvinfo.rcv_sid, data->rcv_sid);
            CHECK_EQ(result->header.rcvinfo.rcv_ssn, data->rcv_ssn);
            CHECK_EQ(result->header.rcvinfo.rcv_ppid, data->rcv_ppid);
            CHECK_EQ(result->header.rcvinfo.rcv_assoc_id, data->rcv_assoc_id);
          }
          result->header.rcvinfo = *data;
        } break;
        default:
          LOG(INFO) << "\tUnknown type: " << scmsg->cmsg_type;
          break;
      }
    }

    CHECK_NE(last_flags & MSG_CTRUNC, MSG_CTRUNC)
        << ": Control message truncated.";

    CHECK_LE(result->size, max_size_)
        << ": Message overflowed buffer on stream "
        << result->header.rcvinfo.rcv_sid << ".";
  }

  result->partial_deliveries = count - 1;
  if (count > 1) {
    VLOG(1) << "Final count: " << count;
    VLOG(1) << "Final size: " << result->size;
  }

  if ((MSG_NOTIFICATION & last_flags)) {
    result->message_type = Message::kNotification;
  } else {
    result->message_type = Message::kMessage;
  }
  return result;
}

void SctpReadWrite::CloseSocket() {
  if (fd_ == -1) {
    return;
  }
  LOG(INFO) << "close(" << fd_ << ")";
  PCHECK(close(fd_) == 0);
  fd_ = -1;
}

void SctpReadWrite::DoSetMaxSize() {
  // Have the kernel give us a factor of 10 more.  This lets us have more than
  // one full sized packet in flight.
  size_t max_size = max_size_ * 10;

  CHECK_GE(ReadRMemMax(), max_size)
      << "rmem_max is too low. To increase rmem_max temporarily, do sysctl "
         "-w net.core.rmem_max="
      << max_size;
  CHECK_GE(ReadWMemMax(), max_size)
      << "wmem_max is too low. To increase wmem_max temporarily, do sysctl "
         "-w net.core.wmem_max="
      << max_size;
  PCHECK(setsockopt(fd(), SOL_SOCKET, SO_RCVBUF, &max_size, sizeof(max_size)) ==
         0);
  PCHECK(setsockopt(fd(), SOL_SOCKET, SO_SNDBUF, &max_size, sizeof(max_size)) ==
         0);
}

void Message::LogRcvInfo() const {
  LOG(INFO) << "\tSNDRCV (stream=" << header.rcvinfo.rcv_sid
            << " ssn=" << header.rcvinfo.rcv_ssn
            << " tsn=" << header.rcvinfo.rcv_tsn << " flags=0x" << std::hex
            << header.rcvinfo.rcv_flags << std::dec
            << " ppid=" << header.rcvinfo.rcv_ppid
            << " cumtsn=" << header.rcvinfo.rcv_cumtsn << ")";
}

size_t ReadRMemMax() {
  struct stat current_stat;
  if (stat("/proc/sys/net/core/rmem_max", &current_stat) != -1) {
    return static_cast<size_t>(
        std::stoi(util::ReadFileToStringOrDie("/proc/sys/net/core/rmem_max")));
  } else {
    LOG(WARNING) << "/proc/sys/net/core/rmem_max doesn't exist.  Are you in a "
                    "container?";
    return 212992;
  }
}

size_t ReadWMemMax() {
  struct stat current_stat;
  if (stat("/proc/sys/net/core/wmem_max", &current_stat) != -1) {
    return static_cast<size_t>(
        std::stoi(util::ReadFileToStringOrDie("/proc/sys/net/core/wmem_max")));
  } else {
    LOG(WARNING) << "/proc/sys/net/core/wmem_max doesn't exist.  Are you in a "
                    "container?";
    return 212992;
  }
}

}  // namespace message_bridge
}  // namespace aos
