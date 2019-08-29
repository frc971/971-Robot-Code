#ifndef AOS_IPC_LIB_QUEUE_RACER_H_
#define AOS_IPC_LIB_QUEUE_RACER_H_

#include <string.h>

#include "aos/ipc_lib/lockless_queue.h"

namespace aos {
namespace ipc_lib {

struct ThreadState;

// Class to test the queue by spinning up a bunch of writing threads and racing
// them together to all write at once.
class QueueRacer {
 public:
  QueueRacer(LocklessQueueMemory *memory, int num_threads,
             uint64_t num_messages, LocklessQueueConfiguration config);

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
    LocklessQueue queue(memory_, config_);
    return queue.LatestQueueIndex().index();
  }

 private:
  // Wipes the queue memory out so we get a clean start.
  void Reset() { memset(memory_, 0, LocklessQueueMemorySize(config_)); }

  // This is a separate method so that when all the ASSERT_* methods, we still
  // clean up all the threads.  Otherwise we get an assert on the way out of
  // RunIteration instead of getting all the way back to gtest.
  void CheckReads(bool race_reads, int write_wrap_count,
                  ::std::vector<ThreadState> *threads);

  LocklessQueueMemory *memory_;
  const uint64_t num_threads_;
  const uint64_t num_messages_;

  // The overall number of writes executed will always be between the two of
  // these.  We can't atomically count writes, so we have to bound them.
  //
  // Number of writes about to be started.
  ::std::atomic<uint64_t> started_writes_;
  // Number of writes completed.
  ::std::atomic<uint64_t> finished_writes_;

  const LocklessQueueConfiguration config_;
};

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_QUEUE_RACER_H_
