#include <stdio.h>

#include "aos/linux_code/init.h"
#include "frc971/actors/horizontal_can_pickup_action.q.h"
#include "frc971/actors/horizontal_can_pickup_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init();

  ::frc971::actors::HorizontalCanPickupActor horizontal_can_pickup(
      &::frc971::actors::horizontal_can_pickup_action);
  horizontal_can_pickup.Run();

  ::aos::Cleanup();
  return 0;
}
