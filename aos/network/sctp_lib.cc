#include "aos/network/sctp_lib.h"

#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/sctp.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <string_view>

#include "aos/util/file.h"

DEFINE_string(interface, "", "network interface");
DEFINE_bool(disable_ipv6, false, "disable ipv6");

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

bool Ipv6Enabled() {
  if (FLAGS_disable_ipv6) {
    return false;
  }
  int fd = socket(AF_INET6, SOCK_SEQPACKET, IPPROTO_SCTP);
  if (fd != -1) {
    close(fd);
    return true;
  }
  switch (errno) {
    case EAFNOSUPPORT:
    case EINVAL:
    case EPROTONOSUPPORT:
      PLOG(INFO) << "no ipv6";
      return false;
    default:
      PLOG(FATAL) << "Open socket failed";
      return false;
  };
}

struct sockaddr_storage ResolveSocket(std::string_view host, int port,
                                      bool use_ipv6) {
  struct sockaddr_storage result;
  struct addrinfo *addrinfo_result;
  struct sockaddr_in *t_addr = (struct sockaddr_in *)&result;
  struct sockaddr_in6 *t_addr6 = (struct sockaddr_in6 *)&result;
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  if (!use_ipv6) {
    hints.ai_family = AF_INET;
  } else {
    // Default to IPv6 as the clearly superior protocol, since it also handles
    // IPv4.
    hints.ai_family = AF_INET6;
  }
  hints.ai_socktype = SOCK_SEQPACKET;
  hints.ai_protocol = IPPROTO_SCTP;
  // We deliberately avoid AI_ADDRCONFIG here because it breaks running things
  // inside Bazel's test sandbox, which has no non-localhost IPv4 or IPv6
  // addresses. Also, it's not really helpful, because most systems will have
  // link-local addresses of both types with any interface that's up.
  hints.ai_flags = AI_PASSIVE | AI_V4MAPPED | AI_NUMERICSERV;
  int ret = getaddrinfo(host.empty() ? nullptr : std::string(host).c_str(),
                        std::to_string(port).c_str(), &hints, &addrinfo_result);
  if (ret == EAI_SYSTEM) {
    PLOG(FATAL) << "getaddrinfo failed to look up '" << host << "'";
  } else if (ret != 0) {
    LOG(FATAL) << "getaddrinfo failed to look up '" << host
               << "': " << gai_strerror(ret);
  }
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
    // with data if enabled. This typically only matters during congestion.
    // However, Linux seems to interleave under memory pressure regardless of
    // this being enabled, so we have to handle it in the code anyways, so might
    // as well turn it on all the time.
    // TODO(Brian): Change this to 2 once we have kernels that support it, and
    // also address the TODO in ProcessNotification to match on all the
    // necessary fields.
    int interleaving = 1;
    PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_FRAGMENT_INTERLEAVE,
                      &interleaving, sizeof(interleaving)) == 0);
  }
  {
    // Enable recvinfo when a packet arrives.
    int on = 1;
    PCHECK(setsockopt(fd_, IPPROTO_SCTP, SCTP_RECVRCVINFO, &on, sizeof(int)) ==
           0);
  }

  {
    // TODO(austin): This is the old style registration...  But, the sctp
    // stack out in the wild for linux is old and primitive.
    struct sctp_event_subscribe subscribe;
    memset(&subscribe, 0, sizeof(subscribe));
    subscribe.sctp_association_event = 1;
    subscribe.sctp_stream_change_event = 1;
    subscribe.sctp_partial_delivery_event = 1;
    PCHECK(setsockopt(fd(), SOL_SCTP, SCTP_EVENTS, (char *)&subscribe,
                      sizeof(subscribe)) == 0);
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

// We read each fragment into a fresh Message, because most of them won't be
// fragmented. If we do end up with a fragment, then we copy the data out of it.
aos::unique_c_ptr<Message> SctpReadWrite::ReadMessage() {
  CHECK(fd_ != -1);

  while (true) {
    aos::unique_c_ptr<Message> result(
        reinterpret_cast<Message *>(malloc(sizeof(Message) + max_size_ + 1)));

    struct msghdr inmessage;
    memset(&inmessage, 0, sizeof(struct msghdr));

    struct iovec iov;
    iov.iov_len = max_size_ + 1;
    iov.iov_base = result->mutable_data();

    inmessage.msg_iov = &iov;
    inmessage.msg_iovlen = 1;

    char incmsg[CMSG_SPACE(sizeof(_sctp_cmsg_data_t))];
    inmessage.msg_control = incmsg;
    inmessage.msg_controllen = sizeof(incmsg);

    inmessage.msg_namelen = sizeof(struct sockaddr_storage);
    inmessage.msg_name = &result->sin;

    const ssize_t size = recvmsg(fd_, &inmessage, MSG_DONTWAIT);
    if (size == -1) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        // These are all non-fatal failures indicating we should retry later.
        return nullptr;
      }
      PLOG(FATAL) << "recvmsg on sctp socket " << fd_ << " failed";
    }

    CHECK(!(inmessage.msg_flags & MSG_CTRUNC))
        << ": Control message truncated.";

    CHECK_LE(size, static_cast<ssize_t>(max_size_))
        << ": Message overflowed buffer on stream "
        << result->header.rcvinfo.rcv_sid << ".";

    result->size = size;
    if (MSG_NOTIFICATION & inmessage.msg_flags) {
      result->message_type = Message::kNotification;
    } else {
      result->message_type = Message::kMessage;
    }
    result->partial_deliveries = 0;

    {
      bool found_rcvinfo = false;
      for (struct cmsghdr *scmsg = CMSG_FIRSTHDR(&inmessage); scmsg != NULL;
           scmsg = CMSG_NXTHDR(&inmessage, scmsg)) {
        switch (scmsg->cmsg_type) {
          case SCTP_RCVINFO: {
            CHECK(!found_rcvinfo);
            found_rcvinfo = true;
            result->header.rcvinfo =
                *reinterpret_cast<struct sctp_rcvinfo *>(CMSG_DATA(scmsg));
          } break;
          default:
            LOG(INFO) << "\tUnknown type: " << scmsg->cmsg_type;
            break;
        }
      }
      CHECK_EQ(found_rcvinfo, result->message_type == Message::kMessage)
          << ": Failed to find a SCTP_RCVINFO cmsghdr. flags: "
          << inmessage.msg_flags;
    }
    if (result->message_type == Message::kNotification) {
      // Notifications are never fragmented, just return it now.
      CHECK(inmessage.msg_flags & MSG_EOR)
          << ": Notifications should never be big enough to fragment";
      if (ProcessNotification(result.get())) {
        // We handled this notification internally, so don't pass it on.
        return nullptr;
      }
      return result;
    }

    auto partial_message_iterator =
        std::find_if(partial_messages_.begin(), partial_messages_.end(),
                     [&result](const aos::unique_c_ptr<Message> &candidate) {
                       return result->header.rcvinfo.rcv_sid ==
                                  candidate->header.rcvinfo.rcv_sid &&
                              result->header.rcvinfo.rcv_ssn ==
                                  candidate->header.rcvinfo.rcv_ssn &&
                              result->header.rcvinfo.rcv_assoc_id ==
                                  candidate->header.rcvinfo.rcv_assoc_id;
                     });
    if (partial_message_iterator != partial_messages_.end()) {
      const aos::unique_c_ptr<Message> &partial_message =
          *partial_message_iterator;
      // Verify it's really part of the same message.
      CHECK_EQ(partial_message->message_type, result->message_type)
          << ": for " << result->header.rcvinfo.rcv_sid << ","
          << result->header.rcvinfo.rcv_ssn << ","
          << result->header.rcvinfo.rcv_assoc_id;
      CHECK_EQ(partial_message->header.rcvinfo.rcv_ppid,
               result->header.rcvinfo.rcv_ppid)
          << ": for " << result->header.rcvinfo.rcv_sid << ","
          << result->header.rcvinfo.rcv_ssn << ","
          << result->header.rcvinfo.rcv_assoc_id;

      // Now copy the data over and update the size.
      CHECK_LE(partial_message->size + result->size, max_size_)
          << ": Assembled fragments overflowed buffer on stream "
          << result->header.rcvinfo.rcv_sid << ".";
      memcpy(partial_message->mutable_data() + partial_message->size,
             result->data(), result->size);
      ++partial_message->partial_deliveries;
      VLOG(1) << "Merged fragment of " << result->size << " after "
              << partial_message->size << ", had "
              << partial_message->partial_deliveries
              << ", for: " << result->header.rcvinfo.rcv_sid << ","
              << result->header.rcvinfo.rcv_ssn << ","
              << result->header.rcvinfo.rcv_assoc_id;
      partial_message->size += result->size;
      result.reset();
    }

    if (inmessage.msg_flags & MSG_EOR) {
      // This is the last fragment, so we have something to return.
      if (partial_message_iterator != partial_messages_.end()) {
        // It was already merged into the message in the list, so now we pull
        // that out of the list and return it.
        CHECK(!result);
        result = std::move(*partial_message_iterator);
        partial_messages_.erase(partial_message_iterator);
        VLOG(1) << "Final count: " << (result->partial_deliveries + 1)
                << ", size: " << result->size
                << ", for: " << result->header.rcvinfo.rcv_sid << ","
                << result->header.rcvinfo.rcv_ssn << ","
                << result->header.rcvinfo.rcv_assoc_id;
      }
      CHECK(result);
      return result;
    }
    if (partial_message_iterator == partial_messages_.end()) {
      VLOG(1) << "Starting fragment for: " << result->header.rcvinfo.rcv_sid
              << "," << result->header.rcvinfo.rcv_ssn << ","
              << result->header.rcvinfo.rcv_assoc_id;
      // Need to record this as the first fragment.
      partial_messages_.emplace_back(std::move(result));
    }
  }
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
  size_t max_size = max_size_;

  // This sets the max packet size that we can send.
  CHECK_GE(ReadWMemMax(), max_size)
      << "wmem_max is too low. To increase wmem_max temporarily, do sysctl "
         "-w net.core.wmem_max="
      << max_size;
  PCHECK(setsockopt(fd(), SOL_SOCKET, SO_SNDBUF, &max_size, sizeof(max_size)) ==
         0);

  // The SO_RCVBUF option (also controlled by net.core.rmem_default) needs to be
  // decently large but the actual size can be measured by tuning.  The defaults
  // should be fine.  If it isn't big enough, transmission will fail.
}

bool SctpReadWrite::ProcessNotification(const Message *message) {
  const union sctp_notification *const snp =
      reinterpret_cast<const union sctp_notification *>(message->data());
  switch (snp->sn_header.sn_type) {
    case SCTP_PARTIAL_DELIVERY_EVENT: {
      const struct sctp_pdapi_event *const partial_delivery =
          &snp->sn_pdapi_event;
      CHECK_EQ(partial_delivery->pdapi_length, sizeof(*partial_delivery))
          << ": Kernel's SCTP code is not a version we support";
      switch (partial_delivery->pdapi_indication) {
        case SCTP_PARTIAL_DELIVERY_ABORTED: {
          const auto iterator = std::find_if(
              partial_messages_.begin(), partial_messages_.end(),
              [partial_delivery](const aos::unique_c_ptr<Message> &candidate) {
                // TODO(Brian): Once we have new enough userpace headers, for
                // kernels that support level-2 interleaving, we'll need to add
                // this:
                //   candidate->header.rcvinfo.rcv_sid ==
                //     partial_delivery->pdapi_stream &&
                //   candidate->header.rcvinfo.rcv_ssn ==
                //     partial_delivery->pdapi_seq &&
                return candidate->header.rcvinfo.rcv_assoc_id ==
                       partial_delivery->pdapi_assoc_id;
              });
          CHECK(iterator != partial_messages_.end())
              << ": Got out of sync with the kernel for "
              << partial_delivery->pdapi_assoc_id;
          VLOG(1) << "Pruning partial delivery for "
                  << iterator->get()->header.rcvinfo.rcv_sid << ","
                  << iterator->get()->header.rcvinfo.rcv_ssn << ","
                  << iterator->get()->header.rcvinfo.rcv_assoc_id;
          partial_messages_.erase(iterator);
        }
          return true;
      }
    } break;
  }
  return false;
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
