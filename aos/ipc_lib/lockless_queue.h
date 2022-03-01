#ifndef AOS_IPC_LIB_LOCKLESS_QUEUE_H_
#define AOS_IPC_LIB_LOCKLESS_QUEUE_H_

#include <sys/signalfd.h>
#include <sys/types.h>

#include <csignal>
#include <optional>
#include <vector>

#include "absl/types/span.h"
#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/data_alignment.h"
#include "aos/ipc_lib/index.h"
#include "aos/time/time.h"
#include "aos/uuid.h"

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

// Structure to hold the state required to pin messages.
struct Pinner {
  // The same as Sender::tid. See there for docs.
  aos_mutex tid;

  // Queue index of the message we have pinned, or Invalid if there isn't one.
  AtomicQueueIndex pinned;

  // This should always be valid.
  //
  // Note that this is fully independent from pinned. It's just a place to stash
  // a message, to ensure there's always an unpinned one for a writer to grab.
  AtomicIndex scratch_index;
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
    monotonic_clock::time_point monotonic_sent_time;
    realtime_clock::time_point realtime_sent_time;
    // Timestamps of the message from the remote node.  These are transparently
    // passed through.
    monotonic_clock::time_point monotonic_remote_time;
    realtime_clock::time_point realtime_remote_time;

    // Queue index from the remote node.
    uint32_t remote_queue_index;

    // Remote boot UUID for this message.
    UUID source_boot_uuid;

    size_t length;
  } header;

  // Returns the start of the data buffer, given that message_data_size is
  // the same one used to allocate this message's memory.
  char *data(size_t message_data_size) {
    return RoundedData(message_data_size);
  }
  const char *data(size_t message_data_size) const {
    return RoundedData(message_data_size);
  }

  // Returns the pre-buffer redzone, given that message_data_size is the same
  // one used to allocate this message's memory.
  absl::Span<char> PreRedzone(size_t message_data_size) {
    char *const end = data(message_data_size);
    const auto result =
        absl::Span<char>(&data_pointer[0], end - &data_pointer[0]);
    DCHECK_LT(result.size(), kChannelDataRedzone + kChannelDataAlignment);
    return result;
  }
  absl::Span<const char> PreRedzone(size_t message_data_size) const {
    const char *const end = data(message_data_size);
    const auto result =
        absl::Span<const char>(&data_pointer[0], end - &data_pointer[0]);
    DCHECK_LT(result.size(), kChannelDataRedzone + kChannelDataAlignment);
    return result;
  }

  // Returns the post-buffer redzone, given that message_data_size is the same
  // one used to allocate this message's memory.
  absl::Span<char> PostRedzone(size_t message_data_size, size_t message_size) {
    DCHECK_LT(message_data_size, message_size);
    char *const redzone_end = reinterpret_cast<char *>(this) + message_size;
    char *const data_end = data(message_data_size) + message_data_size;
    DCHECK_GT(static_cast<void *>(redzone_end), static_cast<void *>(data_end));
    const auto result = absl::Span<char>(data_end, redzone_end - data_end);
    DCHECK_LT(result.size(), kChannelDataRedzone + kChannelDataAlignment * 2);
    return result;
  }
  absl::Span<const char> PostRedzone(size_t message_data_size,
                                     size_t message_size) const {
    DCHECK_LT(message_data_size, message_size);
    const char *const redzone_end =
        reinterpret_cast<const char *>(this) + message_size;
    const char *const data_end = data(message_data_size) + message_data_size;
    DCHECK_GT(static_cast<const void *>(redzone_end),
              static_cast<const void *>(data_end));
    const auto result =
        absl::Span<const char>(data_end, redzone_end - data_end);
    DCHECK_LT(result.size(), kChannelDataRedzone + kChannelDataAlignment * 2);
    return result;
  }

 private:
  // This returns a non-const pointer into a const object. Be very careful
  // about const correctness in publicly accessible APIs using it.
  char *RoundedData(size_t message_data_size) const {
    return RoundChannelData(
        const_cast<char *>(&data_pointer[0] + kChannelDataRedzone),
        message_data_size);
  }

  char data_pointer[];
};

struct LocklessQueueConfiguration {
  // Size of the watchers list.
  size_t num_watchers;
  // Size of the sender list.
  size_t num_senders;
  // Size of the pinner list.
  size_t num_pinners;

  // Size of the list of pointers into the messages list.
  size_t queue_size;
  // Size in bytes of the data stored in each Message.
  size_t message_data_size;

  size_t message_size() const;

  size_t num_messages() const { return num_senders + num_pinners + queue_size; }
};

// Structure to hold the state of the queue.
//
// Reads and writes are lockless and constant time.
//
// Adding a new watcher doesn't need to be constant time for the watcher (this
// is done before the watcher goes RT), but needs to be RT for the sender.
struct LocklessQueueMemory;

// Returns the size of the LocklessQueueMemory.
size_t LocklessQueueMemorySize(LocklessQueueConfiguration config);

// Initializes the queue memory.  memory must be either a valid pointer to the
// queue datastructure, or must be zero initialized.
LocklessQueueMemory *InitializeLocklessQueueMemory(
    LocklessQueueMemory *memory, LocklessQueueConfiguration config);

const static unsigned int kWakeupSignal = SIGRTMIN + 2;

// A convenient wrapper for accessing a lockless queue.
class LocklessQueue {
 public:
  LocklessQueue(const LocklessQueueMemory *const_memory,
                LocklessQueueMemory *memory, LocklessQueueConfiguration config)
      : const_memory_(const_memory), memory_(memory), config_(config) {}

  void Initialize();

  LocklessQueueConfiguration config() const { return config_; }

  const LocklessQueueMemory *const_memory() { return const_memory_; }
  LocklessQueueMemory *memory() { return memory_; }

 private:
  const LocklessQueueMemory *const_memory_;
  LocklessQueueMemory *memory_;
  LocklessQueueConfiguration config_;
};

class LocklessQueueWatcher {
 public:
  LocklessQueueWatcher(const LocklessQueueWatcher &) = delete;
  LocklessQueueWatcher &operator=(const LocklessQueueWatcher &) = delete;
  LocklessQueueWatcher(LocklessQueueWatcher &&other)
      : memory_(other.memory_), watcher_index_(other.watcher_index_) {
    other.watcher_index_ = -1;
  }
  LocklessQueueWatcher &operator=(LocklessQueueWatcher &&other) {
    std::swap(memory_, other.memory_);
    std::swap(watcher_index_, other.watcher_index_);
    return *this;
  }

  ~LocklessQueueWatcher();

  // Registers this thread to receive the kWakeupSignal signal when
  // LocklessQueueWakeUpper::Wakeup is called. Returns nullopt if there was an
  // error in registration.
  // TODO(austin): Change the API if we find ourselves with more errors.
  static std::optional<LocklessQueueWatcher> Make(LocklessQueue queue,
                                                  int priority);

 private:
  LocklessQueueWatcher(LocklessQueueMemory *memory, int priority);

  LocklessQueueMemory *memory_ = nullptr;

  // Index in the watcher list that our entry is, or -1 if no watcher is
  // registered.
  int watcher_index_ = -1;
};

class LocklessQueueWakeUpper {
 public:
  LocklessQueueWakeUpper(LocklessQueue queue);

  // Sends the kWakeupSignal to all threads which have called RegisterWakeup.
  //
  // priority of 0 means nonrt.  nonrt could have issues, so we don't PI boost
  // if nonrt.
  int Wakeup(int current_priority);

 private:
  // Memory and datastructure used to sort a list of watchers to wake
  // up.  This isn't a copy of Watcher since tid is simpler to work with here
  // than the futex above.
  struct WatcherCopy {
    pid_t tid;
    pid_t pid;
    int priority;
  };

  const LocklessQueueMemory *const memory_;
  const int pid_;
  const uid_t uid_;

  ::std::vector<WatcherCopy> watcher_copy_;
};

// Sender for blocks of data.  The resources associated with a sender are
// scoped to this object's lifetime.
class LocklessQueueSender {
 public:
  // Enum of possible sending errors
  // Send returns GOOD if the messages was sent successfully, INVALID_REDZONE if
  // one of a message's redzones has invalid data, or MESSAGES_SENT_TOO_FAST if
  // more than queue_size messages were going to be sent in a
  // channel_storage_duration_.
  enum class Result { GOOD, INVALID_REDZONE, MESSAGES_SENT_TOO_FAST };

  LocklessQueueSender(const LocklessQueueSender &) = delete;
  LocklessQueueSender &operator=(const LocklessQueueSender &) = delete;
  LocklessQueueSender(LocklessQueueSender &&other)
      : memory_(other.memory_),
        sender_index_(other.sender_index_),
        channel_storage_duration_(other.channel_storage_duration_) {
    other.memory_ = nullptr;
    other.sender_index_ = -1;
  }
  LocklessQueueSender &operator=(LocklessQueueSender &&other) {
    std::swap(memory_, other.memory_);
    std::swap(sender_index_, other.sender_index_);
    return *this;
  }

  ~LocklessQueueSender();

  // Creates a sender.  If we couldn't allocate a sender, returns nullopt.
  // TODO(austin): Change the API if we find ourselves with more errors.
  static std::optional<LocklessQueueSender> Make(
      LocklessQueue queue, monotonic_clock::duration channel_storage_duration);

  // Sends a message without copying the data.
  // Copy at most size() bytes of data into the memory pointed to by Data(),
  // and then call Send().
  // Note: calls to Data() are expensive enough that you should cache it.
  size_t size() const;
  void *Data();
  LocklessQueueSender::Result Send(
      size_t length, monotonic_clock::time_point monotonic_remote_time,
      realtime_clock::time_point realtime_remote_time,
      uint32_t remote_queue_index, const UUID &source_boot_uuid,
      monotonic_clock::time_point *monotonic_sent_time = nullptr,
      realtime_clock::time_point *realtime_sent_time = nullptr,
      uint32_t *queue_index = nullptr);

  // Sends up to length data.  Does not wakeup the target.
  LocklessQueueSender::Result Send(
      const char *data, size_t length,
      monotonic_clock::time_point monotonic_remote_time,
      realtime_clock::time_point realtime_remote_time,
      uint32_t remote_queue_index, const UUID &source_boot_uuid,
      monotonic_clock::time_point *monotonic_sent_time = nullptr,
      realtime_clock::time_point *realtime_sent_time = nullptr,
      uint32_t *queue_index = nullptr);

  int buffer_index() const;

 private:
  LocklessQueueSender(LocklessQueueMemory *memory,
                      monotonic_clock::duration channel_storage_duration);

  // Pointer to the backing memory.
  LocklessQueueMemory *memory_ = nullptr;

  // Index into the sender list.
  int sender_index_ = -1;

  // Storage duration of the channel used to check if messages were sent too
  // fast
  const monotonic_clock::duration channel_storage_duration_;
};

std::ostream &operator<<(std::ostream &os, const LocklessQueueSender::Result r);

// Pinner for blocks of data.  The resources associated with a pinner are
// scoped to this object's lifetime.
class LocklessQueuePinner {
 public:
  LocklessQueuePinner(const LocklessQueuePinner &) = delete;
  LocklessQueuePinner &operator=(const LocklessQueuePinner &) = delete;
  LocklessQueuePinner(LocklessQueuePinner &&other)
      : memory_(other.memory_),
        const_memory_(other.const_memory_),
        pinner_index_(other.pinner_index_) {
    other.pinner_index_ = -1;
  }
  LocklessQueuePinner &operator=(LocklessQueuePinner &&other) {
    std::swap(memory_, other.memory_);
    std::swap(const_memory_, other.const_memory_);
    std::swap(pinner_index_, other.pinner_index_);
    return *this;
  }

  ~LocklessQueuePinner();

  // Creates a pinner.  If we couldn't allocate a pinner, returns nullopt.
  // TODO(austin): Change the API if we find ourselves with more errors.
  static std::optional<LocklessQueuePinner> Make(LocklessQueue queue);

  // Attempts to pin the message at queue_index.
  // Un-pins the previous message.
  // Returns the buffer index (non-negative) if it succeeds.
  // Returns -1 if that message is no longer in the queue.
  int PinIndex(uint32_t queue_index);

  // Read at most size() bytes of data into the memory pointed to by Data().
  // Note: calls to Data() are expensive enough that you should cache it.
  // Don't call Data() before a successful PinIndex call.
  size_t size() const;
  const void *Data() const;

 private:
  LocklessQueuePinner(LocklessQueueMemory *memory,
                      const LocklessQueueMemory *const_memory);

  // Pointer to the backing memory.
  LocklessQueueMemory *memory_ = nullptr;
  const LocklessQueueMemory *const_memory_ = nullptr;

  // Index into the pinner list.
  int pinner_index_ = -1;
};

class LocklessQueueReader {
 public:
  enum class Result { TOO_OLD, GOOD, NOTHING_NEW, OVERWROTE };

  LocklessQueueReader(LocklessQueue queue) : memory_(queue.const_memory()) {
    queue.Initialize();
  }

  // If you ask for a queue index 2 past the newest, you will still get
  // NOTHING_NEW until that gets overwritten with new data.  If you ask for an
  // element newer than QueueSize() from the current message, we consider it
  // behind by a large amount and return TOO_OLD.  If the message is modified
  // out from underneath us as we read it, return OVERWROTE.
  //
  // data may be nullptr to indicate the data should not be copied.
  Result Read(uint32_t queue_index,
              monotonic_clock::time_point *monotonic_sent_time,
              realtime_clock::time_point *realtime_sent_time,
              monotonic_clock::time_point *monotonic_remote_time,
              realtime_clock::time_point *realtime_remote_time,
              uint32_t *remote_queue_index, UUID *source_boot_uuid,
              size_t *length, char *data) const;

  // Returns the index to the latest queue message.  Returns empty_queue_index()
  // if there are no messages in the queue.  Do note that this index wraps if
  // more than 2^32 messages are sent.
  QueueIndex LatestIndex() const;

 private:
  const LocklessQueueMemory *const memory_;
};

// Returns the number of messages which are logically in the queue at a time.
size_t LocklessQueueSize(const LocklessQueueMemory *memory);

// Returns the number of bytes queue users are allowed to read/write within each
// message.
size_t LocklessQueueMessageDataSize(const LocklessQueueMemory *memory);

// TODO(austin): Return the oldest queue index.  This lets us catch up nicely
// if we got behind.
// The easiest way to implement this is likely going to be to reserve the
// first modulo of values for the initial time around, and never reuse them.
// That lets us do a simple atomic read of the next index and deduce what has
// happened.  It will involve the simplest atomic operations.

// TODO(austin): Make it so we can find the indices which were sent just
// before and after a time with a binary search.

// Prints to stdout the data inside the queue for debugging.
void PrintLocklessQueueMemory(LocklessQueueMemory *memory);

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_LOCKLESS_QUEUE_H_
