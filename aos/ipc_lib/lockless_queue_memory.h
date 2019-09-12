#ifndef AOS_IPC_LIB_LOCKLESS_QUEUE_MEMORY_H_
#define AOS_IPC_LIB_LOCKLESS_QUEUE_MEMORY_H_

#include <sys/types.h>
#include <atomic>

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/index.h"
#include "aos/time/time.h"

namespace aos {
namespace ipc_lib {

struct LocklessQueueMemory {
  // A lot of things get easier if the only lockless writes are when messages
  // are published.  Do note, the datastructures protected by this lock need to
  // be consistent at all times because a reader (or writer) may read them
  // regardless of if the lock is held or not.
  //
  // Any non-constant time operations need to be done at queue startup time,
  // including cleanup.  Cleanup is done when the next queue is opened.
  aos_mutex queue_setup_lock;
  ::std::atomic<bool> initialized;

  LocklessQueueConfiguration config;

  // Size of the watchers list.
  size_t num_watchers() const { return config.num_watchers; }
  // Size of the sender list.
  size_t num_senders() const { return config.num_senders; }

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

  // There is then memory allocated after this structure.  That memory is used
  // to store the messages, queue, watchers, and senders.  This is equivalent to
  // writing:
  //
  // AtomicIndex queue[config.queue_size];
  // Message messages[config.queue_size + config.num_senders];
  // Watcher watchers[config.num_watchers];
  // Sender senders[config.num_senders];

  // Aligned pointer to where the data starts.
  // Use a 64 bit type to require 64 bit alignment of the data inside.
  uint64_t data[];

  // Memory size functions for all 4 lists.
  size_t SizeOfQueue() { return SizeOfQueue(config); }
  static size_t SizeOfQueue(LocklessQueueConfiguration config) {
    return sizeof(AtomicIndex) * config.queue_size;
  }

  size_t SizeOfMessages() { return SizeOfMessages(config); }
  static size_t SizeOfMessages(LocklessQueueConfiguration config) {
    return config.message_size() * config.num_messages();
  }

  size_t SizeOfWatchers() { return SizeOfWatchers(config); }
  static size_t SizeOfWatchers(LocklessQueueConfiguration config) {
    return sizeof(Watcher) * config.num_watchers;
  }

  size_t SizeOfSenders() { return SizeOfSenders(config); }
  static size_t SizeOfSenders(LocklessQueueConfiguration config) {
    return sizeof(Sender) * config.num_senders;
  }

  // Getters for each of the 4 lists.
  Sender *GetSender(size_t sender_index) {
    return reinterpret_cast<Sender *>(
        reinterpret_cast<uintptr_t>(&data[0]) + SizeOfQueue() +
        SizeOfMessages() + SizeOfWatchers() + sender_index * sizeof(Sender));
  }

  Watcher *GetWatcher(size_t watcher_index) {
    return reinterpret_cast<Watcher *>(reinterpret_cast<uintptr_t>(&data[0]) +
                                       SizeOfQueue() + SizeOfMessages() +
                                       watcher_index * sizeof(Watcher));
  }

  AtomicIndex *GetQueue(uint32_t index) {
    return reinterpret_cast<AtomicIndex *>(
        reinterpret_cast<uintptr_t>(&data[0]) + sizeof(AtomicIndex) * index);
  }

  // There are num_senders + queue_size messages.  The free list is really the
  // sender list, since those are messages available to be filled in and sent.
  // This removes the need to find lost messages when a sender dies.
  Message *GetMessage(Index index) {
    return reinterpret_cast<Message *>(reinterpret_cast<uintptr_t>(&data[0]) +
                                       SizeOfQueue() +
                                       index.message_index() * message_size());
  }


  // Helpers to fetch messages from the queue.
  Index LoadIndex(QueueIndex index) {
    return GetQueue(index.Wrapped())->Load();
  }
  Message *GetMessage(QueueIndex index) {
    return GetMessage(LoadIndex(index));
  }
};

}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_LOCKLESS_QUEUE_MEMORY_H_
