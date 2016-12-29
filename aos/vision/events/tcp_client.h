#ifndef _AOS_VISION_DEBUG_TCP_SERVER_H_
#define _AOS_VISION_DEBUG_TCP_SERVER_H_

#include "aos/vision/events/epoll_events.h"

#include <memory>

namespace aos {
namespace events {

// Handles the client connection logic to hostname:portno
class TcpClient : public EpollEvent {
 public:
  TcpClient(const char *hostname, int portno);

  // Implement ReadEvent from EpollEvent to use this class.
};

}  // namespace events
}  // namespace aos

#endif  // _AOS_VISION_DEBUG_TCP_SERVER_H_
