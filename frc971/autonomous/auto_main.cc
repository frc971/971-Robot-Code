#include "stdio.h"

#include "aos/aos_core.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/auto.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  ::frc971::autonomous::autonomous.FetchLatest();
  while (!::frc971::autonomous::autonomous.get()) {
      ::frc971::autonomous::autonomous.FetchNextBlocking();
  }

  while (true) {
    while (!::frc971::autonomous::autonomous->run_auto) {
      ::frc971::autonomous::autonomous.FetchNextBlocking();
    }
    ::frc971::autonomous::HandleAuto();

    while (::frc971::autonomous::autonomous->run_auto) {
      ::frc971::autonomous::autonomous.FetchNextBlocking();
    }
  }
  ::aos::Cleanup();
  return 0;
}

