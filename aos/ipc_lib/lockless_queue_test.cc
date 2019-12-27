#include "aos/ipc_lib/lockless_queue.h"

#include <inttypes.h>
#include <signal.h>
#include <unistd.h>
#include <wait.h>
#include <chrono>
#include <memory>
#include <random>
#include <thread>

#include "aos/event.h"
#include "aos/events/epoll.h"
#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/queue_racer.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/realtime.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"

DEFINE_int32(min_iterations, 100,
             "Minimum number of stress test iterations to run");
DEFINE_int32(duration, 5, "Number of seconds to test for");
DEFINE_int32(print_rate, 60, "Number of seconds between status prints");

// The roboRIO can only handle 10 threads before exploding.  Set the default for
// ARM to 10.
DEFINE_int32(thread_count,
#if defined(__ARM_EABI__)
             10,
#else
             100,
#endif
             "Number of threads to race");

namespace aos {
namespace ipc_lib {
namespace testing {

namespace chrono = ::std::chrono;

class LocklessQueueTest : public ::testing::Test {
 public:
  LocklessQueueTest() {
    config_.num_watchers = 10;
    config_.num_senders = 100;
    config_.queue_size = 10000;
    // Exercise the alignment code.  This would throw off alignment.
    config_.message_data_size = 101;

    // Since our backing store is an array of uint64_t for alignment purposes,
    // normalize by the size.
    memory_.resize(LocklessQueueMemorySize(config_) / sizeof(uint64_t));

    Reset();
  }

  LocklessQueueMemory *get_memory() {
    return reinterpret_cast<LocklessQueueMemory *>(&(memory_[0]));
  }

  void Reset() { memset(get_memory(), 0, LocklessQueueMemorySize(config_)); }

  // Runs until the signal is received.
  void RunUntilWakeup(Event *ready, int priority) {
    LocklessQueue queue(get_memory(), config_);
    internal::EPoll epoll;
    SignalFd signalfd({kWakeupSignal});

    epoll.OnReadable(signalfd.fd(), [&signalfd, &epoll]() {
      signalfd_siginfo result = signalfd.Read();

      fprintf(stderr, "Got signal: %d\n", result.ssi_signo);
      epoll.Quit();
    });

    // Register to be woken up *after* the signalfd is catching the signals.
    queue.RegisterWakeup(priority);

    // And signal we are now ready.
    ready->Set();

    epoll.Run();

    // Cleanup.
    queue.UnregisterWakeup();
    epoll.DeleteFd(signalfd.fd());
  }

  // Use a type with enough alignment that we are guarenteed that everything
  // will be aligned properly on the target platform.
  ::std::vector<uint64_t> memory_;

  LocklessQueueConfiguration config_;
};

typedef LocklessQueueTest LocklessQueueDeathTest;

// Tests that wakeup doesn't do anything if nothing was registered.
TEST_F(LocklessQueueTest, NoWatcherWakeup) {
  LocklessQueue queue(get_memory(), config_);

  EXPECT_EQ(queue.Wakeup(7), 0);
}

// Tests that wakeup doesn't do anything if a wakeup was registered and then
// unregistered.
TEST_F(LocklessQueueTest, UnregisteredWatcherWakeup) {
  LocklessQueue queue(get_memory(), config_);

  queue.RegisterWakeup(5);
  queue.UnregisterWakeup();

  EXPECT_EQ(queue.Wakeup(7), 0);
}

// Tests that wakeup doesn't do anything if the thread dies.
TEST_F(LocklessQueueTest, DiedWatcherWakeup) {
  LocklessQueue queue(get_memory(), config_);

  ::std::thread([this]() {
    // Use placement new so the destructor doesn't get run.
    ::std::aligned_storage<sizeof(LocklessQueue), alignof(LocklessQueue)>::type
        data;
    LocklessQueue *q = new (&data) LocklessQueue(get_memory(), config_);
    // Register a wakeup.
    q->RegisterWakeup(5);
  }).join();

  EXPECT_EQ(queue.Wakeup(7), 0);
}

struct WatcherState {
  ::std::thread t;
  Event ready;
};

// Tests that too many watchers fails like expected.
TEST_F(LocklessQueueTest, TooManyWatchers) {
  // This is going to be a barrel of monkeys.
  // We need to spin up a bunch of watchers.  But, they all need to be in
  // different threads so they have different tids.
  ::std::vector<WatcherState> queues;
  // Reserve num_watchers WatcherState objects so the pointer value doesn't
  // change out from under us below.
  queues.reserve(config_.num_watchers);

  // Event used to trigger all the threads to unregister.
  Event cleanup;

  // Start all the threads.
  for (size_t i = 0; i < config_.num_watchers; ++i) {
    queues.emplace_back();

    WatcherState *s = &queues.back();
    queues.back().t = ::std::thread([this, &cleanup, s]() {
      LocklessQueue q(get_memory(), config_);
      EXPECT_TRUE(q.RegisterWakeup(0));

      // Signal that this thread is ready.
      s->ready.Set();

      // And wait until we are asked to shut down.
      cleanup.Wait();

      q.UnregisterWakeup();
    });
  }

  // Wait until all the threads are actually going.
  for (WatcherState &w : queues) {
    w.ready.Wait();
  }

  // Now try to allocate another one.  This will fail.
  {
    LocklessQueue queue(get_memory(), config_);
    EXPECT_FALSE(queue.RegisterWakeup(0));
  }

  // Trigger the threads to cleanup their resources, and wait unti they are
  // done.
  cleanup.Set();
  for (WatcherState &w : queues) {
    w.t.join();
  }

  // We should now be able to allocate a wakeup.
  {
    LocklessQueue queue(get_memory(), config_);
    EXPECT_TRUE(queue.RegisterWakeup(0));
    queue.UnregisterWakeup();
  }
}

// Tests that too many watchers dies like expected.
TEST_F(LocklessQueueDeathTest, TooManySenders) {
  EXPECT_DEATH(
      {
        ::std::vector<::std::unique_ptr<LocklessQueue>> queues;
        ::std::vector<LocklessQueue::Sender> senders;
        for (size_t i = 0; i < config_.num_senders + 1; ++i) {
          queues.emplace_back(new LocklessQueue(get_memory(), config_));
          senders.emplace_back(queues.back()->MakeSender());
        }
      },
      "Too many senders");
}

// Now, start 2 threads and have them receive the signals.
TEST_F(LocklessQueueTest, WakeUpThreads) {
  // Confirm that the wakeup signal is in range.
  EXPECT_LE(kWakeupSignal, SIGRTMAX);
  EXPECT_GE(kWakeupSignal, SIGRTMIN);

  LocklessQueue queue(get_memory(), config_);

  // Event used to make sure the thread is ready before the test starts.
  Event ready1;
  Event ready2;

  // Start the thread.
  ::std::thread t1([this, &ready1]() { RunUntilWakeup(&ready1, 5); });
  ::std::thread t2([this, &ready2]() { RunUntilWakeup(&ready2, 4); });

  ready1.Wait();
  ready2.Wait();

  EXPECT_EQ(queue.Wakeup(3), 2);

  t1.join();
  t2.join();

  // Clean up afterwords.  We are pretending to be RT when we are really not.
  // So we will be PI boosted up.
  UnsetCurrentThreadRealtimePriority();
}

// Do a simple send test.
TEST_F(LocklessQueueTest, Send) {
  LocklessQueue queue(get_memory(), config_);

  LocklessQueue::Sender sender = queue.MakeSender();

  // Send enough messages to wrap.
  for (int i = 0; i < 20000; ++i) {
    // Confirm that the queue index makes sense given the number of sends.
    EXPECT_EQ(queue.LatestQueueIndex().index(),
              i == 0 ? LocklessQueue::empty_queue_index().index() : i - 1);

    // Send a trivial piece of data.
    char data[100];
    size_t s = snprintf(data, sizeof(data), "foobar%d", i);
    sender.Send(data, s);

    // Confirm that the queue index still makes sense.  This is easier since the
    // empty case has been handled.
    EXPECT_EQ(queue.LatestQueueIndex().index(), i);

    // Read a result from 5 in the past.
    ::aos::monotonic_clock::time_point monotonic_sent_time;
    ::aos::realtime_clock::time_point realtime_sent_time;
    ::aos::monotonic_clock::time_point monotonic_remote_time;
    ::aos::realtime_clock::time_point realtime_remote_time;
    uint32_t remote_queue_index;
    char read_data[1024];
    size_t length;

    QueueIndex index = QueueIndex::Zero(config_.queue_size);
    if (i - 5 < 0) {
      index = index.DecrementBy(5 - i);
    } else {
      index = index.IncrementBy(i - 5);
    }
    LocklessQueue::ReadResult read_result =
        queue.Read(index.index(), &monotonic_sent_time, &realtime_sent_time,
                   &monotonic_remote_time, &realtime_remote_time,
                   &remote_queue_index, &length, &(read_data[0]));

    // This should either return GOOD, or TOO_OLD if it is before the start of
    // the queue.
    if (read_result != LocklessQueue::ReadResult::GOOD) {
      EXPECT_EQ(read_result, LocklessQueue::ReadResult::TOO_OLD);
    }
  }
}

// Races a bunch of sending threads to see if it all works.
TEST_F(LocklessQueueTest, SendRace) {
  const size_t kNumMessages = 10000 / FLAGS_thread_count;

  ::std::mt19937 generator(0);
  ::std::uniform_int_distribution<> write_wrap_count_distribution(0, 10);
  ::std::bernoulli_distribution race_reads_distribution;
  ::std::bernoulli_distribution wrap_writes_distribution;

  const chrono::seconds print_frequency(FLAGS_print_rate);

  QueueRacer racer(get_memory(), FLAGS_thread_count, kNumMessages, config_);
  const monotonic_clock::time_point start_time =
      monotonic_clock::now();
  const monotonic_clock::time_point end_time =
      start_time + chrono::seconds(FLAGS_duration);

  monotonic_clock::time_point monotonic_now = start_time;
  monotonic_clock::time_point next_print_time = start_time + print_frequency;
  uint64_t messages = 0;
  for (int i = 0; i < FLAGS_min_iterations || monotonic_now < end_time; ++i) {
    bool race_reads = race_reads_distribution(generator);
    int write_wrap_count = write_wrap_count_distribution(generator);
    if (!wrap_writes_distribution(generator)) {
      write_wrap_count = 0;
    }
    EXPECT_NO_FATAL_FAILURE(racer.RunIteration(race_reads, write_wrap_count))
        << ": Running with race_reads: " << race_reads
        << ", and write_wrap_count " << write_wrap_count << " and on iteration "
        << i;

    messages += racer.CurrentIndex();

    monotonic_now = monotonic_clock::now();
    if (monotonic_now > next_print_time) {
      double elapsed_seconds = chrono::duration_cast<chrono::duration<double>>(
                                   monotonic_now - start_time)
                                   .count();
      printf("Finished iteration %d, %f iterations/sec, %f messages/second\n",
             i, i / elapsed_seconds,
             static_cast<double>(messages) / elapsed_seconds);
      next_print_time = monotonic_now + print_frequency;
    }
  }
}

// Send enough messages to wrap the 32 bit send counter.
TEST_F(LocklessQueueTest, WrappedSend) {
  uint64_t kNumMessages = 0x100010000ul;
  QueueRacer racer(get_memory(), 1, kNumMessages, config_);

  const monotonic_clock::time_point start_time = monotonic_clock::now();
  EXPECT_NO_FATAL_FAILURE(racer.RunIteration(false, 0));
  const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
  double elapsed_seconds = chrono::duration_cast<chrono::duration<double>>(
                               monotonic_now - start_time)
                               .count();
  printf("Took %f seconds to write %" PRIu64 " messages, %f messages/s\n",
         elapsed_seconds, kNumMessages,
         static_cast<double>(kNumMessages) / elapsed_seconds);
}

}  // namespace testing
}  // namespace ipc_lib
}  // namespace aos
