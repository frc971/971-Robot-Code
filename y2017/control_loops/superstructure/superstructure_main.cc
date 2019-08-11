#include "y2017/control_loops/superstructure/superstructure.h"

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"

int main() {
  ::aos::InitNRT(true);

  ::aos::ShmEventLoop event_loop;
  ::y2017::control_loops::superstructure::Superstructure superstructure(
      &event_loop);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
