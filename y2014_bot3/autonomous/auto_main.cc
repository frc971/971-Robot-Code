#include <stdio.h>

#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "y2014_bot3/autonomous/auto.q.h"
#include "y2014_bot3/autonomous/auto.h"

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init(-1);

  LOG(INFO, "Auto main started.\n");
  ::y2014_bot3::autonomous::autonomous.FetchLatest();
  while (!::y2014_bot3::autonomous::autonomous.get()) {
    ::y2014_bot3::autonomous::autonomous.FetchNextBlocking();
    LOG(INFO, "Got another auto packet\n");
  }

  while (true) {
    while (!::y2014_bot3::autonomous::autonomous->run_auto) {
      ::y2014_bot3::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Starting auto mode\n");
    ::aos::monotonic_clock::time_point start_time =
        ::aos::monotonic_clock::now();
    ::y2014_bot3::autonomous::HandleAuto();

    auto elapsed_time = ::aos::monotonic_clock::now() - start_time;
    LOG(INFO, "Auto mode exited in %f, waiting for it to finish.\n",
        ::std::chrono::duration_cast<::std::chrono::duration<double>>(
            elapsed_time)
            .count());
    while (::y2014_bot3::autonomous::autonomous->run_auto) {
      ::y2014_bot3::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Waiting for auto to start back up.\n");
  }
  ::aos::Cleanup();
  return 0;
}

