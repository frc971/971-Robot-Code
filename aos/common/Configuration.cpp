#include "Configuration.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#ifdef __VXWORKS__
#include <ifLib.h>
#include <inetLib.h>
#else
#include <ifaddrs.h>
#endif

#include "aos/aos_core.h"
#ifndef __VXWORKS__
#include "aos/common/unique_malloc_ptr.h"
#endif

namespace aos {
namespace configuration {

namespace {

#ifdef __VXWORKS__
const size_t kMaxAddrLength = INET_ADDR_LEN;
#else
// Including the terminating '\0'.
const size_t kMaxAddrLength = 18;
#endif
// Returns whether or not the current IP address is in ownIPAddress.
bool GetOwnIPAddress();

#ifdef __VXWORKS__
// vxworks doesn't have real asprintf.......
static inline int aos_asprintf(size_t max_len, char **strp, const char *fmt, ...) {
  *strp = static_cast<char *>(malloc(max_len));
  if (*strp == NULL) {
    return -1;
  }
  va_list argp;
  va_start(argp, fmt);
  const int r = vsnprintf(*strp, max_len, fmt, argp);
  va_end(argp);
  if (static_cast<uintmax_t>(r) >= max_len) {
    errno = EOVERFLOW;
    free(*strp);
    return -1;
  }
  return r;
}

// 4-slot cRIO: motfec0
// 8-slot cRIO port 1: fec0
// ifShow will show you all of the ones on a cRIO
const char *const kCrioNetInterfaces[] = {"fec0", "motfec0"};
bool GetOwnIPAddress(char *buffer, size_t bufferSize) {
  if (bufferSize < INET_ADDR_LEN) {
    LOG(ERROR, "bufferSize(%jd) isn't >= INET_ADDR_LEN(%jd)\n",
        static_cast<intmax_t>(bufferSize),
        static_cast<intmax_t>(INET_ADDR_LEN));
    return false;
  }
  for (size_t i = 0;
       i < sizeof(kCrioNetInterfaces) / sizeof(kCrioNetInterfaces[0]); ++i) {
    if (ifAddrGet(const_cast<char *>(kCrioNetInterfaces[i]), buffer) != OK) {
      LOG(DEBUG, "ifAddrGet(\"%s\", %p) failed with %d: %s\n",
          kCrioNetInterfaces[i], buffer, errno, strerror(errno));
    } else {
      return true;
    }
  }
  LOG(ERROR, "couldn't get the cRIO's IP address\n");
  return false;
}
#else
static inline int aos_asprintf(size_t, char **strp, const char *fmt, ...) {
  va_list argp;
  va_start(argp, fmt);
  const int r = vasprintf(strp, fmt, argp);
  va_end(argp);
  return r;
}

const char *const kAtomNetInterface = "eth0";
bool GetOwnIPAddress(char *buffer, size_t bufferSize) {
  ifaddrs *addrs;
  if (getifaddrs(&addrs) != 0) {
    LOG(ERROR, "getifaddrs(%p) failed with %d: %s\n", &addrs,
        errno, strerror(errno));
    return false;
  }
  // Smart pointers don't work very well for iterating through a linked list,
  // but it does do a very nice job of making sure that addrs gets freed.
  unique_c_ptr<ifaddrs, freeifaddrs> addrs_deleter(addrs);

  for (; addrs != NULL; addrs = addrs->ifa_next) {
    if (addrs->ifa_addr->sa_family == AF_INET) {
      if (strcmp(kAtomNetInterface, addrs->ifa_name) == 0) {
        const char *ipAddress = inet_ntoa(
            reinterpret_cast<sockaddr_in *>(addrs->ifa_addr)->sin_addr);
        strncpy(buffer, ipAddress, bufferSize);
        return true;
      }
    }
  }
  LOG(ERROR, "couldn't find an AF_INET interface named \"%s\"\n",
      kAtomNetInterface);
  return false;
}
#endif

const char *RawIPAddress(uint8_t last_part) {
  char ownIPAddress[kMaxAddrLength];
  if (!GetOwnIPAddress(ownIPAddress, sizeof(ownIPAddress))) {
    return NULL;
  }
  char *last_dot = strrchr(ownIPAddress, '.');
  if (last_dot == NULL) {
    LOG(ERROR, "can't find any '.'s in \"%s\"\n", ownIPAddress);
    return NULL;
  }
  *last_dot = '\0';

  char *r;
  if (aos_asprintf(kMaxAddrLength, &r, "%s.%hhu",
                   ownIPAddress, last_part) == -1) {
    return NULL;
  }
  return r;
}

}  // namespace

const char *GetIPAddress(NetworkDevice device) {
  switch (device) {
    case NetworkDevice::kAtom:
      return RawIPAddress(179);
    case NetworkDevice::kCRIO:
      return RawIPAddress(2);
  }
  LOG(FATAL, "Unknown network device.");
  return NULL;
}

}  // namespace configuration
}  // namespace aos
