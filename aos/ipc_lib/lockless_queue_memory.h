#ifndef AOS_IPC_LIB_LOCKLESS_QUEUE_MEMORY_H_
#define AOS_IPC_LIB_LOCKLESS_QUEUE_MEMORY_H_

#include <sys/types.h>

#include <cstddef>

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/index.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/time/time.h"

namespace aos {
namespace ipc_lib {

struct LocklessQueueMemory {
  // This is held during initialization. Cleanup after dead processes happens
  // during this initialization process.
  //
  // A lot of things get easier if the only lockless writes are when messages
  // are published.  Do note, the datastructures protected by this lock need to
  // be consistent at all times because a reader (or writer) may read them
  // regardless of if the lock is held or not.
  aos_mutex queue_setup_lock;
  // Tracks that one process has completed initialization once.
  // Only accessed under queue_setup_lock.
  bool initialized;

  LocklessQueueConfiguration config;

  // Size of the watcher list.
  size_t num_watchers() const { return config.num_watchers; }
  // Size of the sender list.
  size_t num_senders() const { return config.num_senders; }
  // Size of the pinner list.
  size_t num_pinners() const { return config.num_pinners; }

  // Number of messages logically in the queue at a time.
  // List of pointers into the messages list.
  size_t queue_size() const { return config.queue_size; }

  // Size in bytes of each Message.
  size_t message_size() const { return config.message_size(); }
  // Size in bytes of the data in each Message.
  size_t message_data_size() const { return config.message_data_size; }

  size_t num_messages() const { return config.num_messages(); }

  // Index in the queue of the latest message.  This is wrapped modulo the
  // queue_size when used to look inside the queue for the message pointer.  It
  // starts out invalid when the queue is empty.
  //
  // A message is valid iff its internal index matches the index in the queue.
  AtomicQueueIndex next_queue_index;

  // All processes using a queue need to be able to send messages to each other.
  // We're going to require they have matching UIDs, which is sufficient to do
  // that.
  uid_t uid;

  // There is then memory allocated after this structure.  That memory is used
  // to store the messages, queue, watchers, and senders.  This is equivalent to
  // writing:
  //
  // AtomicIndex queue[config.queue_size];
  // Message messages[config.num_messages()];
  // Watcher watchers[config.num_watchers];
  // Sender senders[config.num_senders];
  // Pinner pinners[config.num_pinners];

  static constexpr size_t kDataAlignment = alignof(std::max_align_t);

  // Aligned pointer to where the data starts.
  // Make sure it's of a type which can safely alias with the actual datatype,
  // and is aligned enough for any of the structs we will access this memory as.
  alignas(kDataAlignment) char data[];

  // Memory size functions for all 4 lists.
  size_t SizeOfQueue() const { return SizeOfQueue(config); }
  static size_t SizeOfQueue(LocklessQueueConfiguration config) {
    return AlignmentRoundUp(sizeof(AtomicIndex) * config.queue_size);
  }

  size_t SizeOfMessages() const { return SizeOfMessages(config); }
  static size_t SizeOfMessages(LocklessQueueConfiguration config) {
    return AlignmentRoundUp(config.message_size() * config.num_messages());
  }

  size_t SizeOfWatchers() const { return SizeOfWatchers(config); }
  static size_t SizeOfWatchers(LocklessQueueConfiguration config) {
    return AlignmentRoundUp(sizeof(Watcher) * config.num_watchers);
  }

  size_t SizeOfSenders() const { return SizeOfSenders(config); }
  static size_t SizeOfSenders(LocklessQueueConfiguration config) {
    return AlignmentRoundUp(sizeof(Sender) * config.num_senders);
  }

  size_t SizeOfPinners() const { return SizeOfPinners(config); }
  static size_t SizeOfPinners(LocklessQueueConfiguration config) {
    return AlignmentRoundUp(sizeof(Pinner) * config.num_pinners);
  }

  // Getters for each of the lists.

  Pinner *GetPinner(size_t pinner_index) {
    static_assert(alignof(Pinner) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<Pinner *>(
        &data[0] + SizeOfQueue() + SizeOfMessages() + SizeOfWatchers() +
        SizeOfSenders() + pinner_index * sizeof(Pinner));
  }

  const Pinner *GetPinner(size_t pinner_index) const {
    static_assert(alignof(const Pinner) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<const Pinner *>(
        &data[0] + SizeOfQueue() + SizeOfMessages() + SizeOfWatchers() +
        SizeOfSenders() + pinner_index * sizeof(Pinner));
  }

  Sender *GetSender(size_t sender_index) {
    static_assert(alignof(Sender) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<Sender *>(&data[0] + SizeOfQueue() +
                                      SizeOfMessages() + SizeOfWatchers() +
                                      sender_index * sizeof(Sender));
  }

  Watcher *GetWatcher(size_t watcher_index) {
    static_assert(alignof(Watcher) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<Watcher *>(&data[0] + SizeOfQueue() +
                                       SizeOfMessages() +
                                       watcher_index * sizeof(Watcher));
  }

  const Watcher *GetWatcher(size_t watcher_index) const {
    static_assert(alignof(const Watcher) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<const Watcher *>(&data[0] + SizeOfQueue() +
                                             SizeOfMessages() +
                                             watcher_index * sizeof(Watcher));
  }

  AtomicIndex *GetQueue(uint32_t index) {
    static_assert(alignof(AtomicIndex) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<AtomicIndex *>(&data[0] +
                                           sizeof(AtomicIndex) * index);
  }
  const AtomicIndex *GetQueue(uint32_t index) const {
    static_assert(alignof(const AtomicIndex) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<const AtomicIndex *>(&data[0] +
                                                 sizeof(AtomicIndex) * index);
  }

  // There are num_messages() messages.  The free list is really the
  // sender+pinner list, since those are messages available to be filled in.
  // This removes the need to find lost messages when a sender dies.
  Message *GetMessage(Index index) {
    static_assert(alignof(Message) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<Message *>(&data[0] + SizeOfQueue() +
                                       index.message_index() * message_size());
  }
  const Message *GetMessage(Index index) const {
    static_assert(alignof(const Message) <= kDataAlignment,
                  "kDataAlignment is too small");
    return reinterpret_cast<const Message *>(
        &data[0] + SizeOfQueue() + index.message_index() * message_size());
  }

  // Helpers to fetch messages from the queue.
  Index LoadIndex(QueueIndex index) const {
    return GetQueue(index.Wrapped())->Load();
  }
  Message *GetMessage(QueueIndex index) { return GetMessage(LoadIndex(index)); }
  const Message *GetMessage(QueueIndex index) const {
    return GetMessage(LoadIndex(index));
  }

  static constexpr size_t AlignmentRoundUp(size_t in) {
    return (in + kDataAlignment - 1) / kDataAlignment * kDataAlignment;
  }
};

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_LOCKLESS_QUEUE_MEMORY_H_
