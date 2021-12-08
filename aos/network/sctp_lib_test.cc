#include "aos/network/sctp_lib.h"
#include "aos/init.h"
#include "gflags/gflags.h"

DEFINE_string(host, "", "host to resolve");
DEFINE_int32(port, 2977, "port to use");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  struct sockaddr_storage sockaddr = aos::message_bridge::ResolveSocket(
      FLAGS_host, FLAGS_port, aos::message_bridge::Ipv6Enabled());
  LOG(INFO) << "Family " << aos::message_bridge::Family(sockaddr);
  LOG(INFO) << "Address " << aos::message_bridge::Address(sockaddr);
  return 0;
}
