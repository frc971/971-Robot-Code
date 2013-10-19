#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "bot3/autonomous/auto.q.h"
#include "bot3/autonomous/auto.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init();

  ::bot3::autonomous::autonomous.FetchLatest();
  while (!::bot3::autonomous::autonomous.get()) {
    ::bot3::autonomous::autonomous.FetchNextBlocking();
    LOG(INFO, "Got another auto packet\n");
  }

  while (true) {
    while (!::bot3::autonomous::autonomous->run_auto) {
      ::bot3::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Starting auto mode\n");
    ::bot3::autonomous::HandleAuto();

    LOG(INFO, "Auto mode exited, waiting for it to finish.\n");
    while (::bot3::autonomous::autonomous->run_auto) {
      ::bot3::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Waiting for auto to start back up.\n");
  }
  ::aos::Cleanup();
  return 0;
}

