#include "aos/common/logging/logging.h"
#include "aos/linux_code/init.h"

#include "bbb/led.h"

#include "frc971/control_loops/claw/claw.q.h"

using ::frc971::control_loops::claw_queue_group;

int main() {
  ::aos::InitNRT();

  ::bbb::LED claw_zeroed(3);

  while (true) {
    CHECK(claw_queue_group.status.FetchNextBlocking());
    claw_zeroed.Set(claw_queue_group.status->zeroed_for_auto);
  }
}
