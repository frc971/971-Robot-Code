#include "y2014/control_loops/claw/claw.h"

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"

int main() {
  ::aos::Init();
  ::aos::ShmEventLoop event_loop;
  ::y2014::control_loops::ClawMotor claw(&event_loop);
  claw.Run();
  ::aos::Cleanup();
  return 0;
}
