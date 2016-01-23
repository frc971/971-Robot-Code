#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2016/actors/superstructure_action.q.h"
#include "y2016/actors/superstructure_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::y2016::actors::SuperstructureActor superstructure(
      &::y2016::actors::superstructure_action);
  superstructure.Run();

  ::aos::Cleanup();
  return 0;
}
