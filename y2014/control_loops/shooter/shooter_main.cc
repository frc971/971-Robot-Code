#include "y2014/control_loops/shooter/shooter.h"

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"

int main() {
  ::aos::Init();
  ::aos::ShmEventLoop event_loop;
  ::y2014::control_loops::ShooterMotor shooter(&event_loop);
  shooter.Run();
  ::aos::Cleanup();
  return 0;
}
