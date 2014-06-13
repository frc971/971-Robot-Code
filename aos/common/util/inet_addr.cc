#include "aos/common/util/inet_addr.h"

#include <stdlib.h>
#ifndef __VXWORKS__
#include <string.h>

#include "aos/common/byteorder.h"
#else

template<typename T>
T hton(T);

template<uint32_t>
uint32_t hton(uint32_t v) { return v; }

#endif

namespace aos {
namespace util {

const char *MakeIPAddress(const in_addr &base_address,
                          ::aos::NetworkAddress last_segment) {
  in_addr address = base_address;
  SetLastSegment(&address, last_segment);

#ifdef __VXWORKS__
  char *r = static_cast<char *>(malloc(INET_ADDR_LEN));
  inet_ntoa_b(address, r);
  return r;
#else
  return strdup(inet_ntoa(address));
#endif
}

void SetLastSegment(in_addr *address, ::aos::NetworkAddress last_segment) {
  address->s_addr &= ~(hton<uint32_t>(0xFF));
  address->s_addr |= hton<uint32_t>(static_cast<uint8_t>(last_segment));
}

}  // namespace util
}  // namespace aos
