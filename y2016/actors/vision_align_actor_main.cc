#include <stdio.h>

#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "y2016/actors/vision_align_action.q.h"
#include "y2016/actors/vision_align_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::aos::ShmEventLoop event_loop;
  ::y2016::actors::VisionAlignActor vision_align(&event_loop);
  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
