#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>
#include <random>
#include <thread>

#include "gflags/gflags.h"

#include "aos/condition.h"
#include "aos/init.h"
#include "aos/ipc_lib/latency_lib.h"
#include "aos/logging/implementations.h"
#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

DEFINE_int32(seconds, 10, "Duration of the test to run");
DEFINE_int32(
    latency_threshold, 1000,
    "Disable tracing when anything takes more than this many microseoncds");
DEFINE_int32(core, 7, "Core to pin to");
DEFINE_int32(sender_priority, 53, "RT priority to send at");
DEFINE_int32(receiver_priority, 52, "RT priority to receive at");
DEFINE_int32(timer_priority, 51, "RT priority to spin the timer at");

DEFINE_bool(log_latency, false, "If true, log the latency");

const uint32_t kSignalNumber = SIGRTMIN + 1;
const uint32_t kQuitSignalNumber = SIGRTMIN + 2;

namespace chrono = ::std::chrono;

namespace aos {

struct WakeupData {
  Mutex mutex;
  Condition condition;

  WakeupData() : condition(&mutex) {}

  monotonic_clock::time_point wakeup_time = monotonic_clock::epoch();

  bool done = false;
};

void SenderThread(WakeupData *data) {
  const monotonic_clock::time_point end_time =
      monotonic_clock::now() + chrono::seconds(FLAGS_seconds);
  // Standard mersenne_twister_engine seeded with 0
  ::std::mt19937 generator(0);

  // Sleep between 1 and 15 ms.
  ::std::uniform_int_distribution<> distribution(1000, 15000);

  SetCurrentThreadAffinity(MakeCpusetFromCpus({FLAGS_core}));
  SetCurrentThreadRealtimePriority(FLAGS_sender_priority);
  while (true) {
    const monotonic_clock::time_point wakeup_time =
        monotonic_clock::now() + chrono::microseconds(distribution(generator));

    ::std::this_thread::sleep_until(wakeup_time);
    const monotonic_clock::time_point monotonic_now = monotonic_clock::now();

    {
      MutexLocker locker(&data->mutex);
      data->wakeup_time = monotonic_now;
      data->condition.Broadcast();

      if (monotonic_now > end_time) {
        break;
      }
    }
  }

  {
    MutexLocker locker(&data->mutex);
    data->done = true;
    data->condition.Broadcast();
  }

  UnsetCurrentThreadRealtimePriority();
}

void ReceiverThread(WakeupData *data) {
  Tracing t;
  t.Start();

  chrono::nanoseconds max_wakeup_latency = chrono::nanoseconds(0);
  chrono::nanoseconds sum_latency = chrono::nanoseconds(0);
  int latency_count = 0;

  SetCurrentThreadAffinity(MakeCpusetFromCpus({FLAGS_core}));
  SetCurrentThreadRealtimePriority(FLAGS_receiver_priority);
  while (true) {
    chrono::nanoseconds wakeup_latency;
    {
      MutexLocker locker(&data->mutex);
      while (data->wakeup_time == monotonic_clock::epoch() && !data->done) {
        CHECK(!data->condition.Wait());
      }

      const monotonic_clock::time_point monotonic_now = monotonic_clock::now();

      if (data->done) {
        break;
      }

      wakeup_latency = monotonic_now - data->wakeup_time;
      data->wakeup_time = monotonic_clock::epoch();
    }

    sum_latency += wakeup_latency;
    ++latency_count;

    max_wakeup_latency = ::std::max(wakeup_latency, max_wakeup_latency);

    if (wakeup_latency > chrono::microseconds(FLAGS_latency_threshold)) {
      t.Stop();
      AOS_LOG(INFO, "Stopped tracing, latency %" PRId64 "\n",
              static_cast<int64_t>(wakeup_latency.count()));
    }

    if (FLAGS_log_latency) {
      AOS_LOG(INFO, "dt: %8d.%03d\n",
              static_cast<int>(wakeup_latency.count() / 1000),
              static_cast<int>(wakeup_latency.count() % 1000));
    }
  }
  UnsetCurrentThreadRealtimePriority();

  const chrono::nanoseconds average_latency = sum_latency / latency_count;

  AOS_LOG(INFO,
          "Max futex wakeup latency: %d.%03d microseconds, average: %d.%03d "
          "microseconds\n",
          static_cast<int>(max_wakeup_latency.count() / 1000),
          static_cast<int>(max_wakeup_latency.count() % 1000),
          static_cast<int>(average_latency.count() / 1000),
          static_cast<int>(average_latency.count() % 1000));
}

int Main(int /*argc*/, char ** /*argv*/) {
  WakeupData data;

  AOS_LOG(INFO, "Main!\n");
  ::std::thread t([]() {
    TimerThread(monotonic_clock::now() + chrono::seconds(FLAGS_seconds),
                FLAGS_timer_priority);
  });

  ::std::thread st([&data]() { SenderThread(&data); });

  ReceiverThread(&data);

  st.join();
  t.join();
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  return ::aos::Main(argc, argv);
}
