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
  PCHECK(getsockopt(fd, SOL_SCTP, SCTP_STATUS,
                    reinterpret_cast<void *>(&status), &size) == 0);

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

aos::unique_c_ptr<Message> ReadSctpMessage(int fd, int max_size) {
  char incmsg[CMSG_SPACE(sizeof(_sctp_cmsg_data_t))];
  struct iovec iov;
  struct msghdr inmessage;

  memset(&inmessage, 0, sizeof(struct msghdr));

  aos::unique_c_ptr<Message> result(
      reinterpret_cast<Message *>(malloc(sizeof(Message) + max_size + 1)));

  iov.iov_len = max_size + 1;
  iov.iov_base = result->mutable_data();

  inmessage.msg_iov = &iov;
  inmessage.msg_iovlen = 1;

  inmessage.msg_control = incmsg;
  inmessage.msg_controllen = sizeof(incmsg);

  inmessage.msg_namelen = sizeof(struct sockaddr_storage);
  inmessage.msg_name = &result->sin;

  ssize_t size;
  PCHECK((size = recvmsg(fd, &inmessage, 0)) > 0);

  result->size = size;
  if ((MSG_NOTIFICATION & inmessage.msg_flags)) {
    result->message_type = Message::kNotification;
  } else {
    result->message_type = Message::kMessage;
  }

  for (struct cmsghdr *scmsg = CMSG_FIRSTHDR(&inmessage); scmsg != NULL;
       scmsg = CMSG_NXTHDR(&inmessage, scmsg)) {
    switch (scmsg->cmsg_type) {
      case SCTP_RCVINFO: {
        struct sctp_rcvinfo *data = (struct sctp_rcvinfo *)CMSG_DATA(scmsg);
        result->header.rcvinfo = *data;
      } break;
      default:
        LOG(INFO) << "\tUnknown type: " << scmsg->cmsg_type;
        break;
    }
  }

  CHECK_NE(inmessage.msg_flags & MSG_CTRUNC, MSG_CTRUNC)
      << ": Control message truncated.";

  CHECK_LE(size, max_size) << ": Message overflowed buffer on stream "
                           << result->header.rcvinfo.rcv_sid << ".";

  return result;
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
