#if !AOS_DEBUG
#undef NDEBUG
#define NDEBUG
#endif

#include "aos/linux_code/ipc_lib/queue.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <memory>
#include <algorithm>

#include "aos/common/type_traits.h"
#include "aos/linux_code/ipc_lib/core_lib.h"

namespace aos {
namespace {

static_assert(shm_ok<RawQueue>::value,
              "RawQueue instances go into shared memory");

const bool kReadDebug = false;
const bool kWriteDebug = false;
const bool kRefDebug = false;
const bool kFetchDebug = false;
const bool kReadIndexDebug = false;

// The number of extra messages the pool associated with each queue will be able
// to hold (for readers who are slow about freeing them or who leak one when
// they get killed).
const int kExtraMessages = 20;

}  // namespace

constexpr Options<RawQueue>::Option RawQueue::kPeek;
constexpr Options<RawQueue>::Option RawQueue::kFromEnd;
constexpr Options<RawQueue>::Option RawQueue::kNonBlock;
constexpr Options<RawQueue>::Option RawQueue::kBlock;
constexpr Options<RawQueue>::Option RawQueue::kOverride;

// This is what gets stuck in before each queue message in memory. It is always
// allocated aligned to 8 bytes and its size has to maintain that alignment for
// the message that follows immediately.
struct RawQueue::MessageHeader {
  MessageHeader *next;

  // Gets the message header immediately preceding msg.
  static MessageHeader *Get(const void *msg) {
    return reinterpret_cast<MessageHeader *>(__builtin_assume_aligned(
        static_cast<uint8_t *>(const_cast<void *>(msg)) - sizeof(MessageHeader),
        alignof(MessageHeader)));
  }

  int32_t ref_count() const {
    return __atomic_load_n(&ref_count_, __ATOMIC_RELAXED);
  }
  void set_ref_count(int32_t val) {
    __atomic_store_n(&ref_count_, val, __ATOMIC_RELAXED);
  }

  void ref_count_sub() {
    __atomic_sub_fetch(&ref_count_, 1, __ATOMIC_RELAXED);
  }
  void ref_count_add() {
    __atomic_add_fetch(&ref_count_, 1, __ATOMIC_RELAXED);
  }

 private:
  // This gets accessed with atomic instructions without any
  // locks held by various member functions.
  int32_t ref_count_;

  // Padding to make the total size 8 bytes if we have 4-byte pointers or bump
  // it to 16 if a pointer is 8 bytes by itself.
#if __SIZEOF_POINTER__ == 8
#ifdef __clang__
  // Clang is smart enough to realize this is unused, but GCC doesn't like the
  // attribute here...
  __attribute__((unused))
#endif
  char padding[4];
#elif __SIZEOF_POINTER__ == 4
  // No padding needed to get 8 byte total size.
#else
#error Unknown pointer size.
#endif
};

inline int RawQueue::index_add1(int index) {
  // Doing it this way instead of with % is more efficient on ARM.
  int r = index + 1;
  assert(index <= data_length_);
  if (r == data_length_) {
    return 0;
  } else {
    return r;
  }
}

void RawQueue::DecrementMessageReferenceCount(const void *msg) {
  MessageHeader *header = MessageHeader::Get(msg);
  header->ref_count_sub();
  if (kRefDebug) {
    printf("%p ref dec count: %p count=%d\n", this, msg, header->ref_count());
  }

  // The only way it should ever be 0 is if we were the last one to decrement,
  // in which case nobody else should have it around to re-increment it or
  // anything in the middle, so this is safe to do not atomically with the
  // decrement.
  if (header->ref_count() == 0) {
    DoFreeMessage(msg);
  } else {
    assert(header->ref_count() > 0);
  }
}

inline void RawQueue::IncrementMessageReferenceCount(const void *msg) const {
  MessageHeader *const header = MessageHeader::Get(msg);
  header->ref_count_add();
  if (kRefDebug) {
    printf("%p ref inc count: %p\n", this, msg);
  }
}

inline void RawQueue::DoFreeMessage(const void *msg) {
  MessageHeader *header = MessageHeader::Get(msg);
  if (kRefDebug) {
    printf("%p ref free to %p: %p\n", this, recycle_, msg);
  }

  if (__builtin_expect(recycle_ != nullptr, 0)) {
    void *const new_msg = recycle_->GetMessage();
    if (new_msg == nullptr) {
      fprintf(stderr, "queue: couldn't get a message"
              " for recycle queue %p\n", recycle_);
    } else {
      header->ref_count_add();
      if (!recycle_->WriteMessage(const_cast<void *>(msg), kOverride)) {
        fprintf(stderr, "queue: %p->WriteMessage(%p, kOverride) failed."
                " aborting\n", recycle_, msg);
        printf("see stderr\n");
        abort();
      }
      msg = new_msg;
      header = MessageHeader::Get(new_msg);
    }
  }

  // This works around GCC bug 60272 (fixed in 4.8.3).
  // new_next should just get replaced with header->next (and the body of the
  // loop should become empty).
  // The bug is that the store to new_next after the compare/exchange is
  // unconditional but it should only be if it fails, which could mean
  // overwriting what somebody else who preempted us right then changed it to.
  // TODO(brians): Get rid of this workaround once we get a new enough GCC.
  MessageHeader *new_next = __atomic_load_n(&free_messages_, __ATOMIC_RELAXED);
  do {
    header->next = new_next;
  } while (__builtin_expect(
      !__atomic_compare_exchange_n(&free_messages_, &new_next, header, true,
                                   __ATOMIC_RELEASE, __ATOMIC_RELAXED),
      0));
}

void *RawQueue::GetMessage() {
  MessageHeader *header = __atomic_load_n(&free_messages_, __ATOMIC_RELAXED);
  do {
    if (__builtin_expect(header == nullptr, 0)) {
      LOG(FATAL, "overused pool of queue %p (%s)\n", this, name_);
    }
  } while (__builtin_expect(
      !__atomic_compare_exchange_n(&free_messages_, &header, header->next, true,
                                   __ATOMIC_ACQ_REL, __ATOMIC_RELAXED),
      0));
  void *msg = reinterpret_cast<uint8_t *>(header + 1);
  // It might be uninitialized, 0 from a previous use, or 1 from previously
  // being recycled.
  header->set_ref_count(1);
  if (kRefDebug) {
    printf("%p ref alloc: %p\n", this, msg);
  }
  return msg;
}

RawQueue::RawQueue(const char *name, size_t length, int hash, int queue_length)
    : readable_(&data_lock_), writable_(&data_lock_) {
  static_assert(shm_ok<RawQueue::MessageHeader>::value,
                "the whole point is to stick it in shared memory");
  static_assert((sizeof(RawQueue::MessageHeader) % 8) == 0,
                "need to revalidate size/alignent assumptions");

  if (queue_length < 1) {
    LOG(FATAL, "queue length %d of %s needs to be at least 1\n", queue_length,
        name_);
  }

  const size_t name_size = strlen(name) + 1;
  char *temp = static_cast<char *>(shm_malloc(name_size));
  memcpy(temp, name, name_size);
  name_ = temp;
  length_ = length;
  hash_ = hash;
  queue_length_ = queue_length;

  next_ = NULL;
  recycle_ = NULL;

  if (kFetchDebug) {
    printf("initializing name=%s, length=%zd, hash=%d, queue_length=%d\n",
           name, length, hash, queue_length);
  }

  data_length_ = queue_length + 1;
  data_ = static_cast<void **>(shm_malloc(sizeof(void *) * data_length_));
  data_start_ = 0;
  data_end_ = 0;
  messages_ = 0;

  msg_length_ = length + sizeof(MessageHeader);

  // Create all of the messages for the free list and stick them on.
  {
    MessageHeader *previous = nullptr;
    for (int i = 0; i < queue_length + kExtraMessages; ++i) {
      MessageHeader *const message =
          static_cast<MessageHeader *>(shm_malloc(msg_length_));
      free_messages_ = message;
      message->next = previous;
      previous = message;
    }
  }

  readable_waiting_ = false;

  if (kFetchDebug) {
    printf("made queue %s\n", name);
  }
}

RawQueue *RawQueue::Fetch(const char *name, size_t length, int hash,
                    int queue_length) {
  if (kFetchDebug) {
    printf("fetching queue %s\n", name);
  }
  if (mutex_lock(&global_core->mem_struct->queues.lock) != 0) {
    LOG(FATAL, "mutex_lock(%p) failed\n",
        &global_core->mem_struct->queues.lock);
  }
  RawQueue *current = static_cast<RawQueue *>(
      global_core->mem_struct->queues.pointer);
  if (current != NULL) {
    while (true) {
      // If we found a matching queue.
      if (strcmp(current->name_, name) == 0 && current->length_ == length &&
          current->hash_ == hash && current->queue_length_ == queue_length) {
        mutex_unlock(&global_core->mem_struct->queues.lock);
        return current;
      } else {
        if (kFetchDebug) {
          printf("rejected queue %s strcmp=%d target=%s\n", current->name_,
                 strcmp(current->name_, name), name);
        }
      }
      // If this is the last one.
      if (current->next_ == NULL) break;
      current = current->next_;
    }
  }

  RawQueue *r = new (shm_malloc(sizeof(RawQueue)))
      RawQueue(name, length, hash, queue_length);
  if (current == NULL) {  // if we don't already have one
    global_core->mem_struct->queues.pointer = r;
  } else {
    current->next_ = r;
  }

  mutex_unlock(&global_core->mem_struct->queues.lock);
  return r;
}

RawQueue *RawQueue::Fetch(const char *name, size_t length, int hash,
                    int queue_length,
                    int recycle_hash, int recycle_length, RawQueue **recycle) {
  RawQueue *r = Fetch(name, length, hash, queue_length);
  r->recycle_ = Fetch(name, length, recycle_hash, recycle_length);
  if (r == r->recycle_) {
    fprintf(stderr, "queue: r->recycle_(=%p) == r(=%p)\n", r->recycle_, r);
    printf("see stderr\n");
    r->recycle_ = NULL;
    abort();
  }
  *recycle = r->recycle_;
  return r;
}

bool RawQueue::DoWriteMessage(void *msg, Options<RawQueue> options) {
  if (kWriteDebug) {
    printf("queue: %p->WriteMessage(%p, %x)\n", this, msg, options.printable());
  }

  bool signal_readable;

  {
    IPCMutexLocker locker(&data_lock_);
    CHECK(!locker.owner_died());

    int new_end;
    while (true) {
      new_end = index_add1(data_end_);
      // If there is room in the queue right now.
      if (new_end != data_start_) break;
      if (options & kNonBlock) {
        if (kWriteDebug) {
          printf("queue: not blocking on %p. returning false\n", this);
        }
        DecrementMessageReferenceCount(msg);
        return false;
      } else if (options & kOverride) {
        if (kWriteDebug) {
          printf("queue: overriding on %p\n", this);
        }
        // Avoid leaking the message that we're going to overwrite.
        DecrementMessageReferenceCount(data_[data_start_]);
        data_start_ = index_add1(data_start_);
      } else {  // kBlock
        assert(options & kBlock);
        if (kWriteDebug) {
          printf("queue: going to wait for writable_ of %p\n", this);
        }
        CHECK(!writable_.Wait());
      }
    }
    data_[data_end_] = msg;
    ++messages_;
    data_end_ = new_end;

    signal_readable = readable_waiting_;
    readable_waiting_ = false;
  }

  if (signal_readable) {
    if (kWriteDebug) {
      printf("queue: broadcasting to readable_ of %p\n", this);
    }
    readable_.Broadcast();
  } else if (kWriteDebug) {
    printf("queue: skipping broadcast to readable_ of %p\n", this);
  }

  if (kWriteDebug) {
    printf("queue: write returning true on queue %p\n", this);
  }
  return true;
}

inline void RawQueue::ReadCommonEnd() {
  if (is_writable()) {
    if (kReadDebug) {
      printf("queue: %ssignalling writable_ of %p\n",
             writable_start_ ? "not " : "", this);
    }
    if (!writable_start_) writable_.Broadcast();
  }
}

bool RawQueue::ReadCommonStart(Options<RawQueue> options, int *index) {
  while (data_start_ == data_end_ || ((index != NULL) && messages_ <= *index)) {
    if (options & kNonBlock) {
      if (kReadDebug) {
        printf("queue: not going to block waiting on %p\n", this);
      }
      return false;
    } else {  // kBlock
      assert(options & kBlock);
      if (kReadDebug) {
        printf("queue: going to wait for readable_ of %p\n", this);
      }
      readable_waiting_ = true;
      // Wait for a message to become readable.
      CHECK(!readable_.Wait());
      if (kReadDebug) {
        printf("queue: done waiting for readable_ of %p\n", this);
      }
    }
  }
  // We have to check down here because we might have unlocked the mutex while
  // Wait()ing above so this value might have changed.
  writable_start_ = is_writable();
  if (kReadDebug) {
    printf("queue: %p->read(%p) start=%d end=%d writable_start=%d\n",
           this, index, data_start_, data_end_, writable_start_);
  }
  return true;
}

inline int RawQueue::LastMessageIndex() const {
  int pos = data_end_ - 1;
  if (pos < 0) {  // If it wrapped around.
    pos = data_length_ - 1;
  }
  return pos;
}

const void *RawQueue::DoReadMessage(Options<RawQueue> options) {
  // TODO(brians): Test this function.
  if (kReadDebug) {
    printf("queue: %p->ReadMessage(%x)\n", this, options.printable());
  }
  void *msg = NULL;

  IPCMutexLocker locker(&data_lock_);
  CHECK(!locker.owner_died());

  if (!ReadCommonStart(options, nullptr)) {
    if (kReadDebug) {
      printf("queue: %p common returned false\n", this);
    }
    return NULL;
  }

  if (options & kFromEnd) {
    if (options & kPeek) {
      if (kReadDebug) {
        printf("queue: %p shortcutting c2: %d\n", this, LastMessageIndex());
      }
      msg = data_[LastMessageIndex()];
      IncrementMessageReferenceCount(msg);
    } else {
      while (true) {
        if (kReadDebug) {
          printf("queue: %p start of c2\n", this);
        }
        // This loop pulls each message out of the buffer.
        const int pos = data_start_;
        data_start_ = index_add1(data_start_);
        // If this is the last one.
        if (data_start_ == data_end_) {
          if (kReadDebug) {
            printf("queue: %p reading from c2: %d\n", this, pos);
          }
          msg = data_[pos];
          break;
        }
        // This message is not going to be in the queue any more.
        DecrementMessageReferenceCount(data_[pos]);
      }
    }
  } else {
    if (kReadDebug) {
      printf("queue: %p reading from d2: %d\n", this, data_start_);
    }
    msg = data_[data_start_];
    if (options & kPeek) {
      IncrementMessageReferenceCount(msg);
    } else {
      data_start_ = index_add1(data_start_);
    }
  }
  ReadCommonEnd();
  if (kReadDebug) {
    printf("queue: %p read returning %p\n", this, msg);
  }
  return msg;
}

const void *RawQueue::DoReadMessageIndex(Options<RawQueue> options,
                                         int *index) {
  if (kReadDebug) {
    printf("queue: %p->ReadMessageIndex(%x, %p(*=%d))\n",
           this, options.printable(), index, *index);
  }
  void *msg = NULL;

  IPCMutexLocker locker(&data_lock_);
  CHECK(!locker.owner_died());

  if (!ReadCommonStart(options, index)) {
    if (kReadDebug) {
      printf("queue: %p common returned false\n", this);
    }
    return NULL;
  }

  // TODO(parker): Handle integer wrap on the index.

  if (options & kFromEnd) {
    if (kReadDebug) {
      printf("queue: %p reading from c1: %d\n", this, LastMessageIndex());
    }
    msg = data_[LastMessageIndex()];

    // We'd skip this if we had kPeek, but kPeek | kFromEnd isn't valid for
    // reading with an index.
    *index = messages_;
  } else {
    // Where we're going to start reading.
    int my_start;

    const int unread_messages = messages_ - *index;
    assert(unread_messages > 0);
    int current_messages = data_end_ - data_start_;
    if (current_messages < 0) current_messages += data_length_;
    if (kReadIndexDebug) {
      printf("queue: %p start=%d end=%d current=%d\n",
             this, data_start_, data_end_, current_messages);
    }
    assert(current_messages > 0);
    // If we're behind the available messages.
    if (unread_messages > current_messages) {
      // Catch index up to the last available message.
      *index = messages_ - current_messages;
      // And that's the one we're going to read.
      my_start = data_start_;
      if (kReadIndexDebug) {
        printf("queue: %p jumping ahead to message %d (have %d) (at %d)\n",
               this, *index, messages_, data_start_);
      }
    } else {
      // Just start reading at the first available message that we haven't yet
      // read.
      my_start = data_end_ - unread_messages;
      if (kReadIndexDebug) {
        printf("queue: %p original read from %d\n", this, my_start);
      }
      if (data_start_ < data_end_) {
        assert(my_start >= 0);
      }
      if (my_start < 0) my_start += data_length_;
    }

    if (kReadDebug) {
      printf("queue: %p reading from d1: %d\n", this, my_start);
    }
    // We have to be either after the start or before the end, even if the queue
    // is wrapped around (should be both if it's not).
    assert((my_start >= data_start_) || (my_start < data_end_));
    // More sanity checking.
    assert((my_start >= 0) && (my_start < data_length_));
    msg = data_[my_start];
    if (!(options & kPeek)) ++(*index);
  }
  IncrementMessageReferenceCount(msg);

  ReadCommonEnd();
  return msg;
}

int RawQueue::FreeMessages() const {
  int r = 0;
  MessageHeader *header = free_messages_;
  while (header != nullptr) {
    ++r;
    header = header->next;
  }
  return r;
}

}  // namespace aos
