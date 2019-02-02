#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"

int main() {
  ::aos::Init();
  ::aos::ShmEventLoop event_loop;
  ::y2019::control_loops::superstructure::Superstructure superstructure(
      &event_loop);
  superstructure.Run();
  ::aos::Cleanup();
  return 0;
}
