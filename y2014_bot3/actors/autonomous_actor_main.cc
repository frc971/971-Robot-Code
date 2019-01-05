#include <stdio.h>

#include "aos/init.h"
#include "frc971/autonomous/auto.q.h"
#include "y2014_bot3/actors/autonomous_actor.h"

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init(-1);

  ::y2014_bot3::actors::AutonomousActor autonomous(
      &::frc971::autonomous::autonomous_action);
  autonomous.Run();

  ::aos::Cleanup();
  return 0;
}
