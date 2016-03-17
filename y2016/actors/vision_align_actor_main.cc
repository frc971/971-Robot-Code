#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2016/actors/vision_align_action.q.h"
#include "y2016/actors/vision_align_actor.h"

using ::aos::time::Time;

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::y2016::actors::VisionAlignActor vision_align(
      &::y2016::actors::vision_align_action);
  vision_align.Run();

  ::aos::Cleanup();
  return 0;
}
