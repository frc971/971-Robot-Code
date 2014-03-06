#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "frc971/actions/selfcatch_action.q.h"
#include "frc971/actions/self_catch_action.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init();

  frc971::actions::SelfCatchAction selfcatch(
      &::frc971::actions::selfcatch_action);
  selfcatch.Run();

  ::aos::Cleanup();
  return 0;
}

