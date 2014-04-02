#ifndef AOS_COMMON_UTIL_INET_ADDR_H_
#define AOS_COMMON_UTIL_INET_ADDR_H_

#ifdef __VXWORKS__
#include <inetLib.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "aos/common/network_port.h"

namespace aos {
namespace util {

// Makes an IP address string from base_address with the last byte set to
// last_segment.
// Returns a malloc(3)ed string.
const char *MakeIPAddress(const in_addr &base_address,
                          ::aos::NetworkAddress last_segment);

// Sets the last byte of *address to last_segment.
void SetLastSegment(in_addr *address, ::aos::NetworkAddress last_segment);

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_INET_ADDR_H_
