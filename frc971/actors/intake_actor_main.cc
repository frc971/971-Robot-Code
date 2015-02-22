#include <stdio.h>

#include "aos/linux_code/init.h"
#include "frc971/actors/intake_action.q.h"
#include "frc971/actors/intake_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init();

  ::frc971::actors::IntakeActor intake(&::frc971::actors::intake_action);
  intake.Run();

  ::aos::Cleanup();
  return 0;
}
