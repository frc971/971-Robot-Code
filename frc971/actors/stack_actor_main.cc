#include <stdio.h>

#include "aos/linux_code/init.h"
#include "frc971/actors/stack_action.q.h"
#include "frc971/actors/stack_actor.h"

int main(int /*argc*/, char* /*argv*/ []) {
  ::aos::Init();

  ::frc971::actors::StackActor stack(&::frc971::actors::stack_action);
  stack.Run();

  ::aos::Cleanup();
  return 0;
}
