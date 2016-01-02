#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2015/actors/horizontal_can_pickup_action.q.h"
#include "y2015/actors/horizontal_can_pickup_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::frc971::actors::HorizontalCanPickupActor horizontal_can_pickup(
      &::frc971::actors::horizontal_can_pickup_action);
  horizontal_can_pickup.Run();

  ::aos::Cleanup();
  return 0;
}
