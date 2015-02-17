#include <stdio.h>

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "frc971/actors/fridge_profile_action.q.h"
#include "frc971/actors/fridge_profile_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  frc971::actors::FridgeProfileActor fridge_profile(
      &::frc971::actors::fridge_profile_action);
  fridge_profile.Run();

  ::aos::Cleanup();
  return 0;
}
