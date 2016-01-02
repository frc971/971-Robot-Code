#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2015/actors/can_pickup_action.q.h"
#include "y2015/actors/can_pickup_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::frc971::actors::CanPickupActor can_pickup(&::frc971::actors::can_pickup_action);
  can_pickup.Run();

  ::aos::Cleanup();
  return 0;
}
