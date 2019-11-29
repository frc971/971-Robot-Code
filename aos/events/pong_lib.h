#ifndef AOS_EVENTS_PONG_LIB_H_
#define AOS_EVENTS_PONG_LIB_H_

#include "aos/events/event_loop.h"
#include "aos/events/pong_generated.h"
#include "aos/events/ping_generated.h"

namespace aos {

// Class which replies to a Ping message with a Pong message immediately.
class Pong {
 public:
  Pong(EventLoop *event_loop);

 private:
  EventLoop *event_loop_;
  aos::Sender<examples::Pong> sender_;
};

}  // namespace aos

#endif  // AOS_EVENTS_PONG_LIB_H_
