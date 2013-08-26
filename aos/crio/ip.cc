#include "aos/crio/ip.h"

#include <ifLib.h>
#include <stdio.h>

namespace aos {
namespace util {

// 4-slot cRIO: motfec0
// 8-slot cRIO port 1: fec0
// `ifShow` will show you all of the ones on a given cRIO
const char *const kCrioNetInterfaces[] = {"fec0", "motfec0"};
in_addr GetOwnIPAddress() {
  char buffer[INET_ADDR_LEN];
  in_addr r;
  while (true) {
    for (size_t i = 0;
         i < sizeof(kCrioNetInterfaces) / sizeof(kCrioNetInterfaces[0]); ++i) {
      if (ifAddrGet(const_cast<char *>(kCrioNetInterfaces[i]), buffer) == OK) {
        if (inet_aton(buffer, &r) == OK) {
          return r;
        } else {
          buffer[sizeof(buffer) - 1] = '\0';
          printf("inet_aton('%s', %p) failed\n", buffer, &r);
        }
      }
    }
  }
}

}  // namespace util
}  // namespace aos
