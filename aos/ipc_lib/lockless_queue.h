#ifndef AOS_IPC_LIB_LOCKLESS_QUEUE_H_
#define AOS_IPC_LIB_LOCKLESS_QUEUE_H_

#include <signal.h>
#include <sys/signalfd.h>
#include <sys/types.h>
#include <vector>

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/index.h"
#include "aos/time/time.h"

namespace aos {
namespace ipc_lib {

// Structure to hold the state required to wake a watcher.
struct Watcher {
  // Mutex that the watcher locks.  If the futex is 0 (or FUTEX_OWNER_DIED),
  // then this watcher is invalid.  The futex variable will then hold the tid of
  // the watcher, or FUTEX_OWNER_DIED if the task died.
  //
  // Note: this is only modified with the queue_setup_lock lock held, but may
  // always be read.
  // Any state modification should happen before the lock is acquired.
  aos_mutex tid;

  // PID of the watcher.
  std::atomic<pid_t> pid;

  // RT priority of the watcher.
  std::atomic<int> priority;
};

// Structure to hold the state required to send messages.
struct Sender {
  // Mutex that the sender locks.  If the futex is 0 (or FUTEX_OWNER_DIED), then
  // this sender is invalid.  The futex variable will then hold the tid of the
  // sender, or FUTEX_OWNER_DIED if the task died.
  //
  // Note: this is only modified with the queue_setup_lock lock held, but may
  // always be read.
  aos_mutex tid;

  // Index of the message we will be filling out.
  AtomicIndex scratch_index;

  // Index of the element being swapped with scratch_index, or Invalid if there
  // is nothing to do.
  AtomicIndex to_replace;
};

// Structure representing a message.
struct Message {
  struct Header {
    // Index of this message in the queue.  Needs to match the index this
    // message is written into the queue at.  The data in this message is only
    // valid if it matches the index in the queue both before and after all the
    // data is read.
    //
    // Note: a value of 0xffffffff always means that the contents aren't valid.
    AtomicQueueIndex queue_index;

    // Timestamp of the message.  Needs to be monotonically incrementing in the
    // queue, which means that time needs to be re-sampled every time a write
    // fails.
    ::aos::monotonic_clock::time_point monotonic_sent_time;
    ::aos::realtime_clock::time_point realtime_sent_time;
    // Timestamps of the message from the remote node.  These are transparently
    // passed through.
    ::aos::monotonic_clock::time_point monotonic_remote_time;
    ::aos::realtime_clock::time_point realtime_remote_time;

    // Queue index from the remote node.
    uint32_t remote_queue_index;

    size_t length;
  } header;

  char data[];
};

struct LocklessQueueConfiguration {
  // Size of the watchers list.
  size_t num_watchers;
  // Size of the sender list.
  size_t num_senders;

  // Size of the list of pointers into the messages list.
  size_t queue_size;
  // Size in bytes of the data stored in each Message.
  size_t message_data_size;

  size_t message_size() const;

  size_t num_messages() const { return num_senders + queue_size; }
};

// Structure to hold the state of the queue.
//
// Reads and writes are lockless and constant time.
//
// Adding a new watcher doesn't need to be constant time for the watcher (this
// is done before the watcher goes RT), but needs to be RT for the sender.
struct LocklessQueueMemory;

// Initializes the queue memory.  memory must be either a valid pointer to the
// queue datastructure, or must be zero initialized.
LocklessQueueMemory *InitializeLocklessQueueMemory(
    LocklessQueueMemory *memory, LocklessQueueConfiguration config);

// Returns the size of the LocklessQueueMemory.
size_t LocklessQueueMemorySize(LocklessQueueConfiguration config);

// Prints to stdout the data inside the queue for debugging.
void PrintLocklessQueueMemory(LocklessQueueMemory *memory);

const static unsigned int kWakeupSignal = SIGRTMIN + 2;

// Class to manage sending and receiving data in the lockless queue.  This is
// separate from the actual memory backing the queue so that memory can be
// managed with mmap to share across the process boundary.
class LocklessQueue {
 public:
  LocklessQueue(LocklessQueueMemory *memory, LocklessQueueConfiguration config);
  LocklessQueue(const LocklessQueue &) = delete;
  LocklessQueue &operator=(const LocklessQueue &) = delete;

  ~LocklessQueue();

  // Returns the number of messages in the queue.
  size_t QueueSize() const;

  size_t message_data_size() const;

  // Registers this thread to receive the kWakeupSignal signal when Wakeup is
  // called. Returns false if there was an error in registration.
  bool RegisterWakeup(int priority);
  // Unregisters the wakeup.
  void UnregisterWakeup();

  // Sends the kWakeupSignal to all threads which have called RegisterWakeup.
  //
  // priority of 0 means nonrt.  nonrt could have issues, so we don't PI boost
  // if nonrt.
  int Wakeup(int current_priority);

  // If you ask for a queue index 2 past the newest, you will still get
  // NOTHING_NEW until that gets overwritten with new data.  If you ask for an
  // element newer than QueueSize() from the current message, we consider it
  // behind by a large amount and return TOO_OLD.  If the message is modified
  // out from underneath us as we read it, return OVERWROTE.
  enum class ReadResult { TOO_OLD, GOOD, NOTHING_NEW, OVERWROTE };
  ReadResult Read(uint32_t queue_index,
                  ::aos::monotonic_clock::time_point *monotonic_sent_time,
                  ::aos::realtime_clock::time_point *realtime_sent_time,
                  ::aos::monotonic_clock::time_point *monotonic_remote_time,
                  ::aos::realtime_clock::time_point *realtime_remote_time,
                  uint32_t *remote_queue_index, size_t *length, char *data);

  // Returns the index to the latest queue message.  Returns empty_queue_index()
  // if there are no messages in the queue.  Do note that this index wraps if
  // more than 2^32 messages are sent.
  QueueIndex LatestQueueIndex();
  static QueueIndex empty_queue_index() { return QueueIndex::Invalid(); }

  // Returns the size of the queue.  This is mostly useful for manipulating
  // QueueIndex.
  size_t queue_size() const;

  // TODO(austin): Return the oldest queue index.  This lets us catch up nicely
  // if we got behind.
  // The easiest way to implement this is likely going to be to reserve the
  // first modulo of values for the initial time around, and never reuse them.
  // That lets us do a simple atomic read of the next index and deduce what has
  // happened.  It will involve the simplest atomic operations.

  // TODO(austin): Make it so we can find the indices which were sent just
  // before and after a time with a binary search.

  // Sender for blocks of data.  The resources associated with a sender are
  // scoped to this object's lifetime.
  class Sender {
   public:
    Sender(const Sender &) = delete;
    Sender &operator=(const Sender &) = delete;
    Sender(Sender &&other)
        : memory_(other.memory_), sender_index_(other.sender_index_) {
      other.memory_ = nullptr;
      other.sender_index_ = -1;
    }
    Sender &operator=(Sender &&other) {
      memory_ = other.memory_;
      sender_index_ = other.sender_index_;
      other.memory_ = nullptr;
      other.sender_index_ = -1;
      return *this;
    }

    ~Sender();

    // Sends a message without copying the data.
    // Copy at most size() bytes of data into the memory pointed to by Data(),
    // and then call Send().
    // Note: calls to Data() are expensive enough that you should cache it.
    size_t size();
    void *Data();
    void Send(size_t length,
              aos::monotonic_clock::time_point monotonic_remote_time =
                  aos::monotonic_clock::min_time,
              aos::realtime_clock::time_point realtime_remote_time =
                  aos::realtime_clock::min_time,
              uint32_t remote_queue_index = 0xffffffff,
              aos::monotonic_clock::time_point *monotonic_sent_time = nullptr,
              aos::realtime_clock::time_point *realtime_sent_time = nullptr,
              uint32_t *queue_index = nullptr);

    // Sends up to length data.  Does not wakeup the target.
    void Send(const char *data, size_t length,
              aos::monotonic_clock::time_point monotonic_remote_time =
                  aos::monotonic_clock::min_time,
              aos::realtime_clock::time_point realtime_remote_time =
                  aos::realtime_clock::min_time,
              uint32_t remote_queue_index = 0xffffffff,
              aos::monotonic_clock::time_point *monotonic_sent_time = nullptr,
              aos::realtime_clock::time_point *realtime_sent_time = nullptr,
              uint32_t *queue_index = nullptr);

   private:
    friend class LocklessQueue;

    Sender(LocklessQueueMemory *memory);

    // Pointer to the backing memory.
    LocklessQueueMemory *memory_ = nullptr;

    // Index into the sender list.
    int sender_index_ = -1;
  };

  // Creates a sender.
  Sender MakeSender();

 private:
  LocklessQueueMemory *memory_ = nullptr;

  // Memory and datastructure used to sort a list of watchers to wake
  // up.  This isn't a copy of Watcher since tid is simpler to work with here
  // than the futex above.
  struct WatcherCopy {
    pid_t tid;
    pid_t pid;
    int priority;
  };
  // TODO(austin): Don't allocate this memory if we aren't going to send.
  ::std::vector<WatcherCopy> watcher_copy_;

  // Index in the watcher list that our entry is, or -1 if no watcher is
  // registered.
  int watcher_index_ = -1;

  const int pid_;
  const uid_t uid_;
};

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_LOCKLESS_QUEUE_H_
