#include "aos/ipc_lib/latency_lib.h"

#include <chrono>
#include <random>
#include <thread>

#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

namespace aos {

namespace chrono = std::chrono;

void TimerThread(const monotonic_clock::time_point end_time,
                 int timer_priority) {
  // Standard mersenne_twister_engine seeded with 0
  ::std::mt19937 generator(0);

  // Sleep between 1 and 15 ms.
  ::std::uniform_int_distribution<> distribution(1000, 15000);

  chrono::nanoseconds max_wakeup_latency = chrono::nanoseconds(0);

  SetCurrentThreadRealtimePriority(timer_priority);
  while (true) {
    const monotonic_clock::time_point wakeup_time =
        monotonic_clock::now() + chrono::microseconds(distribution(generator));

    ::std::this_thread::sleep_until(wakeup_time);
    const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
    const chrono::nanoseconds wakeup_latency = monotonic_now - wakeup_time;

    max_wakeup_latency = ::std::max(wakeup_latency, max_wakeup_latency);

    if (monotonic_now > end_time) {
      break;
    }
  }
  AOS_LOG(INFO, "Max wakeup latency: %d.%d microseconds\n",
          static_cast<int>(max_wakeup_latency.count() / 1000),
          static_cast<int>(max_wakeup_latency.count() % 1000));
}

}  // namespace aos
