#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2016/actors/autonomous_actor.h"

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init(-1);

  ::y2016::actors::AutonomousActor autonomous(
      &::frc971::autonomous::autonomous_action);
  autonomous.Run();

  ::aos::Cleanup();
  return 0;
}
