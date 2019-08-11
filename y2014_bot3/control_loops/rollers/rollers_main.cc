#include "y2014_bot3/control_loops/rollers/rollers.h"

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"

int main() {
  ::aos::InitNRT(true);

  ::aos::ShmEventLoop event_loop;
  ::y2014_bot3::control_loops::Rollers rollers(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
