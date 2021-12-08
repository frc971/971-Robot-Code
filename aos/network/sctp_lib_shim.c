#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>

int socket(int domain, int type, int protocol) {
  static int (*libsocket)(int domain, int type, int protocol) = NULL;
  const char *error;
  if (!libsocket) {
    libsocket = dlsym(RTLD_NEXT, "socket");
    if ((error = dlerror()) != NULL) {
      fprintf(stderr, "shim socket: %s\n", error);
      exit(1);
    }
  }

  if (getenv("has_ipv6")[0] != 'y' && domain == AF_INET6) {
    errno = EAFNOSUPPORT;
    return -1;
  }
  // Force AF_INET since we don't actually know whether this system
  // supports IPv6 and we're just trying to create a socket for the
  // caller to immediately close again.
  return libsocket(AF_INET, type, protocol);
}
