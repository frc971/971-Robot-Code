#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2016_bot3/actors/autonomous_action.q.h"
#include "y2016_bot3/actors/autonomous_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/ []) {
  ::aos::Init(-1);

  ::y2016_bot3::actors::AutonomousActor autonomous(
      &::y2016_bot3::actors::autonomous_action);
  autonomous.Run();

  ::aos::Cleanup();
  return 0;
}
