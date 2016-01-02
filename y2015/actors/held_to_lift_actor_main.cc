#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2015/actors/held_to_lift_action.q.h"
#include "y2015/actors/held_to_lift_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::frc971::actors::HeldToLiftActor lift(
      &::frc971::actors::held_to_lift_action);
  lift.Run();

  ::aos::Cleanup();
  return 0;
}
