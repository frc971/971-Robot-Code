#include <stdio.h>

#include "aos/linux_code/init.h"
#include "frc971/actors/score_action.q.h"
#include "frc971/actors/score_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init();

  frc971::actors::ScoreActor score(&::frc971::actors::score_action);
  score.Run();

  ::aos::Cleanup();
  return 0;
}
