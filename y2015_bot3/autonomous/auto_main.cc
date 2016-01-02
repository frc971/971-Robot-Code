#include <stdio.h>

#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "y2015_bot3/autonomous/auto.q.h"
#include "y2015_bot3/autonomous/auto.h"

using ::aos::time::Time;

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init(-1);

  LOG(INFO, "Auto main started\n");
  ::y2015_bot3::autonomous::autonomous.FetchLatest();
  while (!::y2015_bot3::autonomous::autonomous.get()) {
    ::y2015_bot3::autonomous::autonomous.FetchNextBlocking();
    LOG(INFO, "Got another auto packet\n");
  }

  while (true) {
    while (!::y2015_bot3::autonomous::autonomous->run_auto) {
      ::y2015_bot3::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Starting auto mode\n");
    ::aos::time::Time start_time = ::aos::time::Time::Now();
    ::y2015_bot3::autonomous::HandleAuto();

    ::aos::time::Time elapsed_time = ::aos::time::Time::Now() - start_time;
    LOG(INFO, "Auto mode exited in %f, waiting for it to finish.\n",
        elapsed_time.ToSeconds());
    while (::y2015_bot3::autonomous::autonomous->run_auto) {
      ::y2015_bot3::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Waiting for auto to start back up.\n");
  }
  ::aos::Cleanup();
  return 0;
}

