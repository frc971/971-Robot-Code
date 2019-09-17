#include "gflags/gflags.h"

#include <signal.h>
#include <sys/signalfd.h>
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

// This is a demo program which uses Real-Time posix signals to communicate.
// It measures both latency of a random timer thread, and latency of the
// signals.
//
// To enable function graph:
//   echo "function_graph" > current_tracer

DEFINE_bool(sender, true, "If true, send signals to the other process.");
DEFINE_int32(other_pid, -1, "PID of other process to ping");
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

void SenderThread() {
  const monotonic_clock::time_point end_time =
      monotonic_clock::now() + chrono::seconds(FLAGS_seconds);
  // Standard mersenne_twister_engine seeded with 0
  ::std::mt19937 generator(0);

  // Sleep between 1 and 15 ms.
  ::std::uniform_int_distribution<> distribution(1000, 15000);

  int pid = getpid();
  if (FLAGS_other_pid != -1) {
    pid = FLAGS_other_pid;
  }
  AOS_LOG(INFO, "Current PID: %d\n", pid);

  PinCurrentThreadToCPU(FLAGS_core);
  SetCurrentThreadRealtimePriority(FLAGS_sender_priority);
  while (true) {
    const monotonic_clock::time_point wakeup_time =
        monotonic_clock::now() + chrono::microseconds(distribution(generator));

    ::std::this_thread::sleep_until(wakeup_time);
    const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
    sigval s;
    s.sival_int = static_cast<uint32_t>(
        static_cast<uint64_t>(monotonic_now.time_since_epoch().count()) &
        0xfffffffful);

    PCHECK(sigqueue(pid, kSignalNumber, s));

    if (monotonic_now > end_time) {
      break;
    }
  }

  {
    sigval s;
    s.sival_int = 0;
    PCHECK(sigqueue(pid, kQuitSignalNumber, s));
  }
  UnsetCurrentThreadRealtimePriority();
}

void ReceiverThread() {
  int signalfd_fd;
  Tracing t;
  t.Start();

  sigset_t x;
  sigemptyset(&x);
  sigaddset(&x, kSignalNumber);
  sigaddset(&x, kQuitSignalNumber);

  PCHECK(signalfd_fd = signalfd(-1, &x, SFD_NONBLOCK | SFD_CLOEXEC));
  chrono::nanoseconds max_wakeup_latency = chrono::nanoseconds(0);

  chrono::nanoseconds sum_latency = chrono::nanoseconds(0);
  int latency_count = 0;

  internal::EPoll epoll;

  epoll.OnReadable(signalfd_fd, [&t, &epoll, &max_wakeup_latency, &sum_latency,
                                 &latency_count, signalfd_fd]() {
    const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
    signalfd_siginfo si;
    const int ret =
        read(signalfd_fd, static_cast<void *>(&si), sizeof(signalfd_siginfo));
    CHECK_EQ(ret, static_cast<int>(sizeof(signalfd_siginfo)));

    if (si.ssi_signo == kQuitSignalNumber) {
      epoll.Quit();
      return;
    }

    int64_t wakeup_latency_int64 =
        static_cast<int64_t>(monotonic_now.time_since_epoch().count()) &
        0xfffffffful;

    wakeup_latency_int64 -= static_cast<int64_t>(si.ssi_int);

    if (wakeup_latency_int64 > 0x80000000l) {
      wakeup_latency_int64 -= 0x100000000l;
    }

    const chrono::nanoseconds wakeup_latency(wakeup_latency_int64);

    sum_latency += wakeup_latency;
    ++latency_count;

    max_wakeup_latency = ::std::max(wakeup_latency, max_wakeup_latency);

    if (wakeup_latency > chrono::microseconds(FLAGS_latency_threshold)) {
      t.Stop();
      AOS_LOG(INFO, "Stopped tracing, latency %" PRId64 "\n",
              static_cast<int64_t>(wakeup_latency.count()));
    }

    if (FLAGS_log_latency) {
      AOS_LOG(INFO, "signo: %d, sending pid: %d, dt: %8d.%03d\n", si.ssi_signo,
              si.ssi_pid, static_cast<int>(wakeup_latency_int64 / 1000),
              static_cast<int>(wakeup_latency_int64 % 1000));
    }
  });

  PinCurrentThreadToCPU(FLAGS_core);
  SetCurrentThreadRealtimePriority(FLAGS_receiver_priority);
  epoll.Run();
  UnsetCurrentThreadRealtimePriority();
  epoll.DeleteFd(signalfd_fd);

  const chrono::nanoseconds average_latency = sum_latency / latency_count;

  AOS_LOG(INFO,
          "Max signal wakeup latency: %d.%03d microseconds, average: %d.%03d "
          "microseconds\n",
          static_cast<int>(max_wakeup_latency.count() / 1000),
          static_cast<int>(max_wakeup_latency.count() % 1000),
          static_cast<int>(average_latency.count() / 1000),
          static_cast<int>(average_latency.count() % 1000));

  PCHECK(close(signalfd_fd));
}

int Main(int /*argc*/, char ** /*argv*/) {
  sigset_t x;
  sigemptyset(&x);
  sigaddset(&x, kSignalNumber);
  sigaddset(&x, kQuitSignalNumber);
  pthread_sigmask(SIG_BLOCK, &x, NULL);

  AOS_LOG(INFO, "Main!\n");
  ::std::thread t([]() {
    TimerThread(monotonic_clock::now() + chrono::seconds(FLAGS_seconds),
                FLAGS_timer_priority);
  });

  ::std::thread st([]() { SenderThread(); });

  ReceiverThread();
  st.join();

  t.join();
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  ::aos::logging::Init();
  ::aos::logging::SetImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));

  return ::aos::Main(argc, argv);
}
