#include "aos/linux_code/configuration.h"

#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"
#include "aos/common/unique_malloc_ptr.h"
#include "aos/common/once.h"

namespace aos {
namespace configuration {
namespace {

// Including the terminating '\0'.
const size_t kMaxAddrLength = 18;

// TODO(brians): Don't hard-code this.
const char *const kLinuxNetInterface = "eth0";
const in_addr *DoGetOwnIPAddress() {
  ifaddrs *addrs;
  if (getifaddrs(&addrs) != 0) {
    LOG(FATAL, "getifaddrs(%p) failed with %d: %s\n", &addrs,
        errno, strerror(errno));
  }
  // Smart pointers don't work very well for iterating through a linked list,
  // but it does do a very nice job of making sure that addrs gets freed.
  unique_c_ptr<ifaddrs, freeifaddrs> addrs_deleter(addrs);

  for (; addrs != NULL; addrs = addrs->ifa_next) {
    if (addrs->ifa_addr->sa_family == AF_INET) {
      if (strcmp(kLinuxNetInterface, addrs->ifa_name) == 0) {
        static const in_addr r =
            reinterpret_cast<sockaddr_in *>(__builtin_assume_aligned(
                addrs->ifa_addr, alignof(sockaddr_in)))->sin_addr;
        return &r;
      }
    }
  }
  LOG(FATAL, "couldn't find an AF_INET interface named \"%s\"\n",
      kLinuxNetInterface);
}

const char *DoGetRootDirectory() {
  ssize_t size = 0;
  char *r = NULL;
  while (true) {
    if (r != NULL) delete r;
    size += 256;
    r = new char[size];

    ssize_t ret = readlink("/proc/self/exe", r, size);
    if (ret < 0) {
      if (ret != -1) {
        LOG(WARNING, "it returned %zd, not -1\n", ret);
      }
      LOG(FATAL, "readlink(\"/proc/self/exe\", %p, %zu) failed with %d: %s\n",
          r, size, errno, strerror(errno));
    }
    if (ret < size) {
      void *last_slash = memrchr(r, '/', size);
      if (last_slash == NULL) {
        r[ret] = '\0';
        LOG(FATAL, "couldn't find a '/' in \"%s\"\n", r);
      }
      *static_cast<char *>(last_slash) = '\0';
      LOG(INFO, "got a root dir of \"%s\"\n", r);
      return r;
    }
  }
}

const char *DoGetLoggingDirectory() {
  static const char kSuffix[] = "/../../tmp/robot_logs";
  const char *root = GetRootDirectory();
  char *r = new char[strlen(root) + sizeof(kSuffix)];
  strcpy(r, root);
  strcat(r, kSuffix);
  return r;
}

}  // namespace

const char *GetRootDirectory() {
  static aos::Once<const char> once(DoGetRootDirectory);
  return once.Get();
}

const char *GetLoggingDirectory() {
  static aos::Once<const char> once(DoGetLoggingDirectory);
  return once.Get();
}

const in_addr &GetOwnIPAddress() {
  static aos::Once<const in_addr> once(DoGetOwnIPAddress);
  return *once.Get();
}

}  // namespace configuration
}  // namespace aos
