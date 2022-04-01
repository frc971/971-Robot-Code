#ifndef AOS_IPC_LIB_QUEUE_RACER_H_
#define AOS_IPC_LIB_QUEUE_RACER_H_

#include <cstring>

#include "aos/ipc_lib/lockless_queue.h"

namespace aos {
namespace ipc_lib {

struct ThreadState;

struct QueueRacerConfiguration {
  // Number of threads that send messages
  const int num_threads;
  // Number of messages sent by each thread
  const uint64_t num_messages;
  // Allows QueueRacer to check for multiple returns from calling Send()
  const std::vector<LocklessQueueSender::Result> expected_send_results = {
      LocklessQueueSender::Result::GOOD};
  // Channel Storage Duration for queue used by QueueRacer
  const monotonic_clock::duration channel_storage_duration =
      std::chrono::nanoseconds(1);
  // Set to true if all writes and reads are expected to be successful
  // This allows QueueRacer to be used for checking failure scenarios
  const bool check_writes_and_reads;
};

// Class to test the queue by spinning up a bunch of writing threads and racing
// them together to all write at once.
class QueueRacer {
 public:
  QueueRacer(LocklessQueue queue, int num_threads, uint64_t num_messages);
  QueueRacer(LocklessQueue queue, const QueueRacerConfiguration &config);

  // Runs an iteration of the race.
  //
  // This spins up num_threads, each of which sends num_messages.  These must
  // both be able to fit in the queue without wrapping.
  //
  // Then, this reads back all the messages and confirms that all were received
  // in order, and none were missed.
  //
  // If race_reads is set, start reading (and retry if data isn't ready yet)
  // while writes are still happening.
  //
  // If wrap_writes is nonzero, write enough to overwrite old data.  This
  // necesitates a loser check at the end.
  //
  // If both are set, run an even looser test.
  void RunIteration(bool race_reads, int write_wrap_count);

  size_t CurrentIndex() {
    return LocklessQueueReader(queue_).LatestIndex().index();
  }

 private:
  // Wipes the queue memory out so we get a clean start.
  void Reset() {
    memset(queue_.memory(), 0, LocklessQueueMemorySize(queue_.config()));
  }

  // This is a separate method so that when all the ASSERT_* methods, we still
  // clean up all the threads.  Otherwise we get an assert on the way out of
  // RunIteration instead of getting all the way back to gtest.
  void CheckReads(bool race_reads, int write_wrap_count,
                  ::std::vector<ThreadState> *threads);

  LocklessQueue queue_;
  const uint64_t num_threads_;
  const uint64_t num_messages_;
  const monotonic_clock::duration channel_storage_duration_;
  // Allows QueueRacer to check for multiple returns from calling Send()
  const std::vector<LocklessQueueSender::Result> expected_send_results_;
  const bool check_writes_and_reads_;
  // The overall number of writes executed will always be between the two of
  // these.  We can't atomically count writes, so we have to bound them.
  //
  // Number of writes about to be started.
  ::std::atomic<uint64_t> started_writes_;
  // Number of writes completed.
  ::std::atomic<uint64_t> finished_writes_;
};

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_QUEUE_RACER_H_
