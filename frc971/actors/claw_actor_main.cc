#include <stdio.h>

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "frc971/actors/claw_action.q.h"
#include "frc971/actors/claw_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  frc971::actors::ClawActor claw(
      &::frc971::actors::claw_action);
  claw.Run();

  ::aos::Cleanup();
  return 0;
}
