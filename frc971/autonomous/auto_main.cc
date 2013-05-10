#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/auto.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  ::frc971::autonomous::autonomous.FetchLatest();
  while (!::frc971::autonomous::autonomous.get()) {
    ::frc971::autonomous::autonomous.FetchNextBlocking();
    LOG(INFO, "Got another auto packet\n");
  }

  while (true) {
    while (!::frc971::autonomous::autonomous->run_auto) {
      ::frc971::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Starting auto mode\n");
    ::frc971::autonomous::HandleAuto();

    LOG(INFO, "Auto mode exited, waiting for it to finish.\n");
    while (::frc971::autonomous::autonomous->run_auto) {
      ::frc971::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Waiting for auto to start back up.\n");
  }
  ::aos::Cleanup();
  return 0;
}

