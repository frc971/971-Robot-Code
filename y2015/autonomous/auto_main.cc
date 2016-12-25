#include <stdio.h>
#include <chrono>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "frc971/autonomous/auto.q.h"
#include "y2015/autonomous/auto.h"

int main(int /*argc*/, char * /*argv*/[]) {
  ::aos::Init(-1);

  LOG(INFO, "Auto main started\n");
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
    ::aos::monotonic_clock::time_point start_time =
        ::aos::monotonic_clock::now();
    ::y2015::autonomous::HandleAuto();

    auto elapsed_time = ::aos::monotonic_clock::now() - start_time;
    LOG(INFO, "Auto mode exited in %f, waiting for it to finish.\n",
        ::std::chrono::duration_cast<::std::chrono::duration<double>>(
            elapsed_time)
            .count());
    while (::frc971::autonomous::autonomous->run_auto) {
      ::frc971::autonomous::autonomous.FetchNextBlocking();
      LOG(INFO, "Got another auto packet\n");
    }
    LOG(INFO, "Waiting for auto to start back up.\n");
  }
  ::aos::Cleanup();
  return 0;
}

