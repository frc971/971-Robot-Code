#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2015/actors/pickup_action.q.h"
#include "y2015/actors/pickup_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  frc971::actors::PickupActor pickup(&::frc971::actors::pickup_action);
  pickup.Run();

  ::aos::Cleanup();
  return 0;
}
