#include "aos/common/logging/logging.h"

#include "bbb/led.h"

#include "frc971/control_loops/claw/claw.q.h"

using ::frc971::control_loops::claw_queue_group;

int main() {
  ::bbb::LED claw_zeroed(3);

  while (true) {
    CHECK(claw_queue_group.status.FetchNextBlocking());
    claw_zeroed.Set(claw_queue_group.status->zeroed_for_auto);
  }
}
