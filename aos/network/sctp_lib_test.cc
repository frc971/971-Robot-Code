#include "aos/network/sctp_lib.h"

#include "absl/flags/flag.h"

#include "aos/init.h"

ABSL_FLAG(std::string, host, "", "host to resolve");
ABSL_FLAG(int32_t, port, 2977, "port to use");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  struct sockaddr_storage sockaddr = aos::message_bridge::ResolveSocket(
      absl::GetFlag(FLAGS_host), absl::GetFlag(FLAGS_port),
      aos::message_bridge::Ipv6Enabled());
  LOG(INFO) << "Family " << aos::message_bridge::Family(sockaddr);
  LOG(INFO) << "Address " << aos::message_bridge::Address(sockaddr);
  return 0;
}
