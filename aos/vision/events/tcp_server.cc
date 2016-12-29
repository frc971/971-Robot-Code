#include "aos/vision/events/tcp_server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace events {

namespace {

int MakeSocketNonBlocking(int sfd) {
  int flags;

  PCHECK(flags = fcntl(sfd, F_GETFL, 0));

  flags |= O_NONBLOCK;
  PCHECK(fcntl(sfd, F_SETFL, flags));

  return 0;
}
}  // namespace

// This is all copied from somewhere.
int TCPServerBase::SocketBindListenOnPort(int portno) {
  int parentfd;                  /* parent socket */
  struct sockaddr_in serveraddr; /* server's addr */
  int optval;                    /* flag value for setsockopt */

  /*
   * socket: create the parent socket
   */
  PCHECK(parentfd = socket(AF_INET, SOCK_STREAM, 0));

  /* setsockopt: Handy debugging trick that lets
   * us rerun the server immediately after we kill it;
   * otherwise we have to wait about 20 secs.
   * Eliminates "ERROR on binding: Address already in use" error.
   */
  optval = 1;
  PCHECK(setsockopt(parentfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval,
                    sizeof(int)));

  /*
   * build the server's Internet address
   */
  bzero((char *)&serveraddr, sizeof(serveraddr));

  /* this is an Internet address */
  serveraddr.sin_family = AF_INET;

  /* Listen on 0.0.0.0 */
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /* this is the port we will listen on */
  serveraddr.sin_port = htons((uint16_t)portno);

  /*
   * bind: associate the parent socket with a port
   */
  PCHECK(bind(parentfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)));

  PCHECK(listen(parentfd, SOMAXCONN));

  LOG(INFO, "connected to port: %d on fd: %d\n", portno, parentfd);
  return parentfd;
}

TCPServerBase::~TCPServerBase() { close(fd()); }

void TCPServerBase::ReadEvent() {
  /* We have a notification on the listening socket, which
   means one or more incoming connections. */
  struct sockaddr in_addr;
  socklen_t in_len;
  int infd;

  in_len = sizeof in_addr;
  infd = accept(fd(), &in_addr, &in_len);
  if (infd == -1) {
    if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
      /* We have processed all incoming
         connections. */
      return;
    } else {
      PLOG(WARNING, "accept");
      return;
    }
  }

  /* Make the incoming socket non-blocking and add it to the
     list of fds to monitor. */
  PCHECK(MakeSocketNonBlocking(infd));

  loop()->Add(Construct(infd));
}

}  // namespace events
}  // namespace aos
