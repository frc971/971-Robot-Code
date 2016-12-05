#include <stdio.h>

#include "aos/linux_code/init.h"
#include "y2015/actors/stack_and_hold_action.q.h"
#include "y2015/actors/stack_and_hold_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init(-1);

  ::y2015::actors::StackAndHoldActor stack_and_hold(
      &::y2015::actors::stack_and_hold_action);
  stack_and_hold.Run();

  ::aos::Cleanup();
  return 0;
}
