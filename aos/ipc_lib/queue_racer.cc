#include "aos/ipc_lib/queue_racer.h"

#include <inttypes.h>
#include <string.h>
#include <limits>

#include "aos/event.h"
#include "gtest/gtest.h"

namespace aos {
namespace ipc_lib {
namespace {

struct ThreadPlusCount {
  int thread;
  uint64_t count;
};

}  // namespace

struct ThreadState {
  ::std::thread thread;
  Event ready;
  uint64_t event_count = ::std::numeric_limits<uint64_t>::max();
};

QueueRacer::QueueRacer(LocklessQueueMemory *memory, int num_threads,
                       uint64_t num_messages, LocklessQueueConfiguration config)
    : memory_(memory),
      num_threads_(num_threads),
      num_messages_(num_messages),
      config_(config) {
  Reset();
}

void QueueRacer::RunIteration(bool race_reads, int write_wrap_count) {
  const bool will_wrap = num_messages_ * num_threads_ *
                             static_cast<uint64_t>(1 + write_wrap_count) >
                         config_.queue_size;

  // Clear out shmem.
  Reset();
  started_writes_ = 0;
  finished_writes_ = 0;

  // Event used to start all the threads processing at once.
  Event run;

  ::std::atomic<bool> poll_index{true};

  // List of threads.
  ::std::vector<ThreadState> threads(num_threads_);

  ::std::thread queue_index_racer([this, &poll_index]() {
    LocklessQueue queue(memory_, config_);

    // Track the number of times we wrap, and cache the modulo.
    uint64_t wrap_count = 0;
    uint32_t last_queue_index = 0;
    const uint32_t max_queue_index =
        QueueIndex::MaxIndex(0xffffffffu, queue.QueueSize());
    while (poll_index) {
      // We want to read everything backwards.  This will give us conservative
      // bounds.  And with enough time and randomness, we will see all the cases
      // we care to see.

      // These 3 numbers look at the same thing, but at different points of time
      // in the process.  The process (essentially) looks like:
      //
      // ++started_writes;
      // ++latest_queue_index;
      // ++finished_writes;
      //
      // We want to check that latest_queue_index is bounded by the number of
      // writes started and finished.  Basically, we can say that
      // finished_writes < latest_queue_index always.  And
      // latest_queue_index < started_writes.  And everything always increases.
      // So, if we let more time elapse between sampling finished_writes and
      // latest_queue_index, we will only be relaxing our bounds, not
      // invalidating the check.  The same goes for started_writes.
      //
      // So, grab them in order.
      const uint64_t finished_writes = finished_writes_.load();
      const QueueIndex latest_queue_index_queue_index =
          queue.LatestQueueIndex();
      const uint64_t started_writes = started_writes_.load();

      const uint32_t latest_queue_index_uint32_t =
          latest_queue_index_queue_index.index();
      uint64_t latest_queue_index = latest_queue_index_uint32_t;

      if (latest_queue_index_queue_index !=
          LocklessQueue::empty_queue_index()) {
        // If we got smaller, we wrapped.
        if (latest_queue_index_uint32_t < last_queue_index) {
          ++wrap_count;
        }
        // And apply it.
        latest_queue_index +=
            static_cast<uint64_t>(max_queue_index) * wrap_count;
        last_queue_index = latest_queue_index_uint32_t;
      }

      // For grins, check that we have always started more than we finished.
      // Should never fail.
      EXPECT_GE(started_writes, finished_writes);

      // If we are at the beginning, the queue needs to always return empty.
      if (started_writes == 0) {
        EXPECT_EQ(latest_queue_index_queue_index,
                  LocklessQueue::empty_queue_index());
        EXPECT_EQ(finished_writes, 0);
      } else {
        if (finished_writes == 0) {
          // Plausible to be at the beginning, in which case we don't have
          // anything to check.
          if (latest_queue_index_queue_index !=
              LocklessQueue::empty_queue_index()) {
            // Otherwise, we have started.  The queue can't have any more
            // entries than this.
            EXPECT_GE(started_writes, latest_queue_index + 1);
          }
        } else {
          EXPECT_NE(latest_queue_index_queue_index,
                    LocklessQueue::empty_queue_index());
          // latest_queue_index is an index, not a count.  So it always reads 1
          // low.
          EXPECT_GE(latest_queue_index + 1, finished_writes);
        }
      }
    }
  });

  // Build up each thread and kick it off.
  int thread_index = 0;
  for (ThreadState &t : threads) {
    if (will_wrap) {
      t.event_count = ::std::numeric_limits<uint64_t>::max();
    } else {
      t.event_count = 0;
    }
    t.thread =
        ::std::thread([this, &t, thread_index, &run, write_wrap_count]() {
          // Build up a sender.
          LocklessQueue queue(memory_, config_);
          LocklessQueue::Sender sender = queue.MakeSender();

          // Signal that we are ready to start sending.
          t.ready.Set();

          // Wait until signaled to start running.
          run.Wait();

          // Gogogo!
          for (uint64_t i = 0;
               i < num_messages_ * static_cast<uint64_t>(1 + write_wrap_count);
               ++i) {
            char data[sizeof(ThreadPlusCount)];
            ThreadPlusCount tpc;
            tpc.thread = thread_index;
            tpc.count = i;

            memcpy(data, &tpc, sizeof(ThreadPlusCount));

            if (i % 0x800000 == 0x100000) {
              fprintf(stderr, "Sent %" PRIu64 ", %f %%\n", i,
                      static_cast<double>(i) /
                          static_cast<double>(num_messages_ *
                                              (1 + write_wrap_count)) *
                          100.0);
            }

            ++started_writes_;
            sender.Send(data, sizeof(ThreadPlusCount));
            ++finished_writes_;
          }
        });
    ++thread_index;
  }

  // Wait until all the threads are ready.
  for (ThreadState &t : threads) {
    t.ready.Wait();
  }

  // And start them racing.
  run.Set();

  // Let all the threads finish before reading if we are supposed to not be
  // racing reads.
  if (!race_reads) {
    for (ThreadState &t : threads) {
      t.thread.join();
    }
    poll_index = false;
    queue_index_racer.join();
  }

  CheckReads(race_reads, write_wrap_count, &threads);

  // Reap all the threads.
  if (race_reads) {
    for (ThreadState &t : threads) {
      t.thread.join();
    }
    poll_index = false;
    queue_index_racer.join();
  }

  // Confirm that the number of writes matches the expected number of writes.
  ASSERT_EQ(num_threads_ * num_messages_ * (1 + write_wrap_count),
            started_writes_);
  ASSERT_EQ(num_threads_ * num_messages_ * (1 + write_wrap_count),
            finished_writes_);

  // And that every thread sent the right number of messages.
  for (ThreadState &t : threads) {
    if (will_wrap) {
      if (!race_reads) {
        // If we are wrapping, there is a possibility that a thread writes
        // everything *before* we can read any of it, and it all gets
        // overwritten.
        ASSERT_TRUE(t.event_count == ::std::numeric_limits<uint64_t>::max() ||
                    t.event_count == (1 + write_wrap_count) * num_messages_)
            << ": Got " << t.event_count << " events, expected "
            << (1 + write_wrap_count) * num_messages_;
      }
    } else {
      ASSERT_EQ(t.event_count, num_messages_);
    }
  }
}

void QueueRacer::CheckReads(bool race_reads, int write_wrap_count,
                            ::std::vector<ThreadState> *threads) {
  // Now read back the results to double check.
  LocklessQueue queue(memory_, config_);

  const bool will_wrap =
      num_messages_ * num_threads_ * (1 + write_wrap_count) > queue.QueueSize();

  monotonic_clock::time_point last_monotonic_sent_time =
      monotonic_clock::epoch();
  uint64_t initial_i = 0;
  if (will_wrap) {
    initial_i = (1 + write_wrap_count) * num_messages_ * num_threads_ -
                queue.QueueSize();
  }

  for (uint64_t i = initial_i;
       i < (1 + write_wrap_count) * num_messages_ * num_threads_; ++i) {
    ::aos::monotonic_clock::time_point monotonic_sent_time;
    ::aos::realtime_clock::time_point realtime_sent_time;
    ::aos::monotonic_clock::time_point monotonic_remote_time;
    ::aos::realtime_clock::time_point realtime_remote_time;
    uint32_t remote_queue_index;
    size_t length;
    char read_data[1024];

    // Handle overflowing the message count for the wrap test.
    const uint32_t wrapped_i = i % static_cast<size_t>(QueueIndex::MaxIndex(
                                       0xffffffffu, queue.QueueSize()));
    LocklessQueue::ReadResult read_result =
        queue.Read(wrapped_i, &monotonic_sent_time, &realtime_sent_time,
                   &monotonic_remote_time, &realtime_remote_time,
                   &remote_queue_index, &length, &(read_data[0]));

    if (race_reads) {
      if (read_result == LocklessQueue::ReadResult::NOTHING_NEW) {
        --i;
        continue;
      }
    }

    if (race_reads && will_wrap) {
      if (read_result == LocklessQueue::ReadResult::TOO_OLD) {
        continue;
      }
    }
    // Every message should be good.
    ASSERT_EQ(read_result, LocklessQueue::ReadResult::GOOD) << ": i is " << i;

    // And, confirm that time never went backwards.
    ASSERT_GT(monotonic_sent_time, last_monotonic_sent_time);
    last_monotonic_sent_time = monotonic_sent_time;

    EXPECT_EQ(monotonic_remote_time, aos::monotonic_clock::min_time);
    EXPECT_EQ(realtime_remote_time, aos::realtime_clock::min_time);

    ThreadPlusCount tpc;
    ASSERT_EQ(length, sizeof(ThreadPlusCount));
    memcpy(&tpc, read_data + queue.message_data_size() - length,
           sizeof(ThreadPlusCount));

    if (will_wrap) {
      // The queue won't chang out from under us, so we should get some amount
      // of the tail end of the messages from a a thread.
      // Confirm that once we get our first message, they all show up.
      if ((*threads)[tpc.thread].event_count ==
          ::std::numeric_limits<uint64_t>::max()) {
        (*threads)[tpc.thread].event_count = tpc.count;
      }

      if (race_reads) {
        // Make sure nothing goes backwards.  Really not much we can do here.
        ASSERT_LE((*threads)[tpc.thread].event_count, tpc.count) << ": Thread "
                                                                 << tpc.thread;
        (*threads)[tpc.thread].event_count = tpc.count;
      } else {
        // Make sure nothing goes backwards.  Really not much we can do here.
        ASSERT_EQ((*threads)[tpc.thread].event_count, tpc.count) << ": Thread "
                                                                 << tpc.thread;
      }
    } else {
      // Confirm that we see every message counter from every thread.
      ASSERT_EQ((*threads)[tpc.thread].event_count, tpc.count) << ": Thread "
                                                               << tpc.thread;
    }
    ++(*threads)[tpc.thread].event_count;
  }
}

}  // namespace ipc_lib
}  // namespace aos
