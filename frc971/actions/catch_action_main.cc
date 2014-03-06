#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "frc971/actions/catch_action.q.h"
#include "frc971/actions/catch_action.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init();

  frc971::actions::CatchAction action_catch(
      &::frc971::actions::catch_action);
  action_catch.Run();

  ::aos::Cleanup();
  return 0;
}

