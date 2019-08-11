#include "y2016/control_loops/shooter/shooter.h"

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"

int main() {
  ::aos::InitNRT(true);

  ::aos::ShmEventLoop event_loop;
  ::y2016::control_loops::shooter::Shooter shooter(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
