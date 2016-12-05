#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2015/actors/score_action.q.h"
#include "y2015/actors/score_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  y2015::actors::ScoreActor score(&::y2015::actors::score_action);
  score.Run();

  ::aos::Cleanup();
  return 0;
}
