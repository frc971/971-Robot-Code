#include "y2014_bot3/control_loops/rollers/rollers.h"

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"

int main() {
  ::aos::Init();
  ::aos::ShmEventLoop event_loop;
  ::y2014_bot3::control_loops::Rollers rollers(&event_loop);
  rollers.Run();
  ::aos::Cleanup();
  return 0;
}
