#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2015/actors/score_action.q.h"
#include "y2015/actors/score_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  frc971::actors::ScoreActor score(&::frc971::actors::score_action);
  score.Run();

  ::aos::Cleanup();
  return 0;
}
