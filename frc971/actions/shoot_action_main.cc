#include <stdio.h>

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "frc971/actions/shoot_action.q.h"
#include "frc971/actions/shoot_action.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  frc971::actions::ShootAction shoot(&::frc971::actions::shoot_action);
  shoot.Run();

  ::aos::Cleanup();
  return 0;
}

