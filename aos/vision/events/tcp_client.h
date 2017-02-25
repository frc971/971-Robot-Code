#ifndef _AOS_VISION_DEBUG_TCP_CLIENT_H_
#define _AOS_VISION_DEBUG_TCP_CLIENT_H_

#include "aos/vision/events/epoll_events.h"

#include <memory>
#include <string>

namespace aos {
namespace events {

// Handles the client connection logic to hostname:portno
class TcpClient : public EpollEvent {
 public:
  TcpClient(const std::string &hostname, int portno);

  // Implement ReadEvent from EpollEvent to use this class.
};

}  // namespace events
}  // namespace aos

#endif  // _AOS_VISION_DEBUG_TCP_CLIENT_H_
