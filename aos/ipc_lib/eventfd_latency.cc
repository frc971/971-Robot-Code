#include <sys/eventfd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>
#include <random>
#include <thread>

#include "aos/events/epoll.h"
#include "aos/init.h"
#include "aos/ipc_lib/latency_lib.h"
#include "aos/logging/implementations.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "gflags/gflags.h"

// This is a demo program which uses named pipes to communicate.
// It measures both latency of a random timer thread, and latency of the
// pipe.

DEFINE_int32(seconds, 10, "Duration of the test to run");
DEFINE_int32(
    latency_threshold, 1000,
    "Disable tracing when anything takes more than this many microseoncds");
DEFINE_int32(core, 7, "Core to pin to");
DEFINE_int32(sender_priority, 53, "RT priority to send at");
DEFINE_int32(receiver_priority, 52, "RT priority to receive at");
DEFINE_int32(timer_priority, 51, "RT priority to spin the timer at");

DEFINE_bool(log_latency, false, "If true, log the latency");

namespace chrono = ::std::chrono;

namespace aos {

void SenderThread(int fd) {
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
    char sent_time_buffer[8];
    memcpy(sent_time_buffer, &monotonic_now, sizeof(sent_time_buffer));
    PCHECK(write(fd, sent_time_buffer, sizeof(sent_time_buffer)));

    if (monotonic_now > end_time) {
      break;
    }
  }

  {
    ::std::this_thread::sleep_for(chrono::milliseconds(100));
    const monotonic_clock::time_point stop_time(chrono::nanoseconds(1));
    char sent_time_buffer[8];
    memcpy(sent_time_buffer, &stop_time, sizeof(sent_time_buffer));
    PCHECK(write(fd, sent_time_buffer, sizeof(sent_time_buffer)));
  }
  UnsetCurrentThreadRealtimePriority();
}

void ReceiverThread(int fd) {
  Tracing t;
  t.Start();

  chrono::nanoseconds max_wakeup_latency = chrono::nanoseconds(0);

  chrono::nanoseconds sum_latency = chrono::nanoseconds(0);
  int latency_count = 0;

  internal::EPoll epoll;

  epoll.OnReadable(fd, [&t, &epoll, &max_wakeup_latency, &sum_latency,
                        &latency_count, fd]() {
    char sent_time_buffer[8];
    const int ret = read(fd, static_cast<void *>(sent_time_buffer),
                         sizeof(sent_time_buffer));
    const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
    CHECK_EQ(ret, 8);

    monotonic_clock::time_point sent_time;
    memcpy(&sent_time, sent_time_buffer, sizeof(sent_time_buffer));

    if (sent_time == monotonic_clock::time_point(chrono::nanoseconds(1))) {
      epoll.Quit();
      return;
    }

    const chrono::nanoseconds wakeup_latency = monotonic_now - sent_time;

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
  });

  SetCurrentThreadAffinity(MakeCpusetFromCpus({FLAGS_core}));
  SetCurrentThreadRealtimePriority(FLAGS_receiver_priority);
  epoll.Run();
  UnsetCurrentThreadRealtimePriority();
  epoll.DeleteFd(fd);

  const chrono::nanoseconds average_latency = sum_latency / latency_count;

  AOS_LOG(INFO,
          "Max eventfd wakeup latency: %d.%03d microseconds, average: %d.%03d "
          "microseconds\n",
          static_cast<int>(max_wakeup_latency.count() / 1000),
          static_cast<int>(max_wakeup_latency.count() % 1000),
          static_cast<int>(average_latency.count() / 1000),
          static_cast<int>(average_latency.count() % 1000));
}

int Main(int /*argc*/, char ** /*argv*/) {
  AOS_LOG(INFO, "Main!\n");
  ::std::thread t([]() {
    TimerThread(monotonic_clock::now() + chrono::seconds(FLAGS_seconds),
                FLAGS_timer_priority);
  });

  int fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  PCHECK(fd);

  ::std::thread st([&fd]() { SenderThread(fd); });

  ReceiverThread(fd);
  st.join();

  PCHECK(close(fd));

  t.join();
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  return ::aos::Main(argc, argv);
}
