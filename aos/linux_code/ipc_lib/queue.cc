#include "aos/linux_code/ipc_lib/queue.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <memory>
#include <algorithm>

#include "aos/common/logging/logging.h"
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

// The number of extra messages the pool associated with each queue will be able
// to hold (for readers who are slow about freeing them or who leak one when
// they get killed).
const int kExtraMessages = 20;

}  // namespace

const int RawQueue::kPeek;
const int RawQueue::kFromEnd;
const int RawQueue::kNonBlock;
const int RawQueue::kBlock;
const int RawQueue::kOverride;

struct RawQueue::MessageHeader {
  int ref_count;
  int index;  // in pool_
  // Gets the message header immediately preceding msg.
  static MessageHeader *Get(const void *msg) {
    return reinterpret_cast<MessageHeader *>(__builtin_assume_aligned(
        static_cast<uint8_t *>(const_cast<void *>(msg)) - sizeof(MessageHeader),
        alignof(MessageHeader)));
  }
  void Swap(MessageHeader *other) {
    MessageHeader temp;
    memcpy(&temp, other, sizeof(temp));
    memcpy(other, this, sizeof(*other));
    memcpy(this, &temp, sizeof(*this));
  }
};
static_assert(shm_ok<RawQueue::MessageHeader>::value,
              "the whole point is to stick it in shared memory");

struct RawQueue::ReadData {
  bool writable_start;
};

// TODO(brians) maybe do this with atomic integer instructions so it doesn't
//   have to lock/unlock pool_lock_
void RawQueue::DecrementMessageReferenceCount(const void *msg) {
  MutexLocker locker(&pool_lock_);
  MessageHeader *header = MessageHeader::Get(msg);
  --header->ref_count;
  assert(header->ref_count >= 0);
  if (kRefDebug) {
    printf("ref_dec_count: %p count=%d\n", msg, header->ref_count);
  }
  if (header->ref_count == 0) {
    DoFreeMessage(msg);
  }
}

RawQueue::RawQueue(const char *name, size_t length, int hash, int queue_length)
    : readable_(&data_lock_), writable_(&data_lock_) {
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
  if (data_length_ < 2) {  // TODO(brians) when could this happen?
    data_length_ = 2;
  }
  data_ = static_cast<void **>(shm_malloc(sizeof(void *) * data_length_));
  data_start_ = 0;
  data_end_ = 0;
  messages_ = 0;

  mem_length_ = queue_length + kExtraMessages;
  pool_length_ = 0;
  messages_used_ = 0;
  msg_length_ = length + sizeof(MessageHeader);
  pool_ = static_cast<MessageHeader **>(
      shm_malloc(sizeof(MessageHeader *) * mem_length_));

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
    return NULL;
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

void RawQueue::DoFreeMessage(const void *msg) {
  MessageHeader *header = MessageHeader::Get(msg);
  if (pool_[header->index] != header) {  // if something's messed up
    fprintf(stderr, "queue: something is very very wrong with queue %p."
            " pool_(=%p)[header->index(=%d)] != header(=%p)\n",
            this, pool_, header->index, header);
    printf("queue: see stderr\n");
    abort();
  }
  if (kRefDebug) {
    printf("ref free: %p\n", msg);
  }
  --messages_used_;

  if (recycle_ != NULL) {
    void *const new_msg = recycle_->GetMessage();
    if (new_msg == NULL) {
      fprintf(stderr, "queue: couldn't get a message"
              " for recycle queue %p\n", recycle_);
    } else {
      // Take a message from recycle_ and switch its
      // header with the one being freed, which effectively
      // switches which queue each message belongs to.
      MessageHeader *const new_header = MessageHeader::Get(new_msg);
      // Also switch the messages between the pools.
      pool_[header->index] = new_header;
      {
        MutexLocker locker(&recycle_->pool_lock_);
        recycle_->pool_[new_header->index] = header;
        // Swap the information in both headers.
        header->Swap(new_header);
        // Don't unlock the other pool until all of its messages are valid.
      }
      // use the header for new_msg which is now for this pool
      header = new_header;
      if (!recycle_->WriteMessage(const_cast<void *>(msg), kOverride)) {
        fprintf(stderr, "queue: %p->WriteMessage(%p, kOverride) failed."
                " aborting\n", recycle_, msg);
        printf("see stderr\n");
        abort();
      }
      msg = new_msg;
    }
  }

  // Where the one we're freeing was.
  int index = header->index;
  header->index = -1;
  if (index != messages_used_) {  // if we're not freeing the one on the end
    // Put the last one where the one we're freeing was.
    header = pool_[index] = pool_[messages_used_];
    // Put the one we're freeing at the end.
    pool_[messages_used_] = MessageHeader::Get(msg);
    // Update the former last one's index.
    header->index = index;
  }
}

bool RawQueue::WriteMessage(void *msg, int options) {
  if (kWriteDebug) {
    printf("queue: %p->WriteMessage(%p, %x)\n", this, msg, options);
  }
  if (msg == NULL || msg < reinterpret_cast<void *>(global_core->mem_struct) ||
      msg > static_cast<void *>((
              reinterpret_cast<char *>(global_core->mem_struct) +
              global_core->size))) {
    fprintf(stderr, "queue: attempt to write bad message %p to %p. aborting\n",
            msg, this);
    printf("see stderr\n");
    abort();
  }
  {
    MutexLocker locker(&data_lock_);
    bool writable_waited = false;

    int new_end;
    while (true) {
      new_end = (data_end_ + 1) % data_length_;
      // If there is room in the queue right now.
      if (new_end != data_start_) break;
      if (options & kNonBlock) {
        if (kWriteDebug) {
          printf("queue: not blocking on %p. returning false\n", this);
        }
        return false;
      } else if (options & kOverride) {
        if (kWriteDebug) {
          printf("queue: overriding on %p\n", this);
        }
        // Avoid leaking the message that we're going to overwrite.
        DecrementMessageReferenceCount(data_[data_start_]);
        data_start_ = (data_start_ + 1) % data_length_;
      } else {  // kBlock
        if (kWriteDebug) {
          printf("queue: going to wait for writable_ of %p\n", this);
        }
        writable_.Wait();
        writable_waited = true;
      }
    }
    data_[data_end_] = msg;
    ++messages_;
    data_end_ = new_end;

    if (kWriteDebug) {
      printf("queue: broadcasting to readable_ of %p\n", this);
    }
    readable_.Broadcast();

    // If we got a signal on writable_ here and it's still writable, then we
    // need to signal the next person in line (if any).
    if (writable_waited && is_writable()) {
      if (kWriteDebug) {
        printf("queue: resignalling writable_ of %p\n", this);
      }
      writable_.Signal();
    }
  }
  if (kWriteDebug) {
    printf("queue: write returning true on queue %p\n", this);
  }
  return true;
}

void RawQueue::ReadCommonEnd(ReadData *read_data) {
  if (is_writable()) {
    if (kReadDebug) {
      printf("queue: %ssignalling writable_ of %p\n",
             read_data->writable_start ? "not " : "", this);
    }
    if (!read_data->writable_start) writable_.Signal();
  }
}
bool RawQueue::ReadCommonStart(int options, int *index, ReadData *read_data) {
  read_data->writable_start = is_writable();
  while (data_start_ == data_end_ || ((index != NULL) && messages_ <= *index)) {
    if (options & kNonBlock) {
      if (kReadDebug) {
        printf("queue: not going to block waiting on %p\n", this);
      }
      return false;
    } else {  // kBlock
      if (kReadDebug) {
        printf("queue: going to wait for readable_ of %p\n", this);
      }
      // Wait for a message to become readable.
      readable_.Wait();
      if (kReadDebug) {
        printf("queue: done waiting for readable_ of %p\n", this);
      }
    }
  }
  if (kReadDebug) {
    printf("queue: %p->read(%p) start=%d end=%d\n", this, index, data_start_,
           data_end_);
  }
  return true;
}
void *RawQueue::ReadPeek(int options, int start) {
  void *ret;
  if (options & kFromEnd) {
    int pos = data_end_ - 1;
    if (pos < 0) {  // if it needs to wrap
      pos = data_length_ - 1;
    }
    if (kReadDebug) {
      printf("queue: %p reading from line %d: %d\n", this, __LINE__, pos);
    }
    ret = data_[pos];
  } else {
    if (kReadDebug) {
      printf("queue: %p reading from line %d: %d\n", this, __LINE__, start);
    }
    ret = data_[start];
  }
  MessageHeader *const header = MessageHeader::Get(ret);
  ++header->ref_count;
  if (kRefDebug) {
    printf("ref inc count: %p\n", ret);
  }
  return ret;
}
const void *RawQueue::ReadMessage(int options) {
  if (kReadDebug) {
    printf("queue: %p->ReadMessage(%x)\n", this, options);
  }
  void *msg = NULL;

  MutexLocker locker(&data_lock_);

  ReadData read_data;
  if (!ReadCommonStart(options, NULL, &read_data)) {
    if (kReadDebug) {
      printf("queue: %p common returned false\n", this);
    }
    return NULL;
  }

  if (options & kPeek) {
    msg = ReadPeek(options, data_start_);
  } else {
    if (options & kFromEnd) {
      while (true) {
        if (kReadDebug) {
          printf("queue: %p start of c2\n", this);
        }
        // This loop pulls each message out of the buffer.
        const int pos = data_start_;
        data_start_ = (data_start_ + 1) % data_length_;
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
    } else {
      if (kReadDebug) {
        printf("queue: %p reading from d2: %d\n", this, data_start_);
      }
      msg = data_[data_start_];
      data_start_ = (data_start_ + 1) % data_length_;
    }
  }
  ReadCommonEnd(&read_data);
  if (kReadDebug) {
    printf("queue: %p read returning %p\n", this, msg);
  }
  return msg;
}
const void *RawQueue::ReadMessageIndex(int options, int *index) {
  if (kReadDebug) {
    printf("queue: %p->ReadMessageIndex(%x, %p(*=%d))\n",
           this, options, index, *index);
  }
  void *msg = NULL;

  MutexLocker locker(&data_lock_);

  ReadData read_data;
  if (!ReadCommonStart(options, index, &read_data)) {
    if (kReadDebug) {
      printf("queue: %p common returned false\n", this);
    }
    return NULL;
  }

  // TODO(parker): Handle integer wrap on the index.

  // Where we're going to start reading.
  int my_start;

  const int unread_messages = messages_ - *index;
  const int current_messages = ::std::abs(data_start_ - data_end_);
  if (unread_messages > current_messages) {  // If we're behind the available messages.
    // Catch index up to the last available message.
    *index = messages_ - current_messages;
    // And that's the one we're going to read.
    my_start = data_start_;
  } else {
    // Just start reading at the first available message that we haven't yet
    // read.
    my_start = (data_start_ + unread_messages - 1) % data_length_;
  }


  if (options & kPeek) {
    msg = ReadPeek(options, my_start);
  } else {
    if (options & kFromEnd) {
      if (kReadDebug) {
        printf("queue: %p start of c1\n", this);
      }
      int pos = data_end_ - 1;
      if (pos < 0) {  // If it wrapped.
        pos = data_length_ - 1;  // Unwrap it.
      }
      if (kReadDebug) {
        printf("queue: %p reading from c1: %d\n", this, pos);
      }
      msg = data_[pos];
      *index = messages_;
    } else {
      if (kReadDebug) {
        printf("queue: %p reading from d1: %d\n", this, my_start);
      }
      // This assert checks that we're either within both endpoints (duh) or
      // outside of both of them (if the queue is wrapped around).
      assert((my_start >= data_start_ && my_start < data_end_) ||
             (my_start > data_end_ && my_start <= data_start_));
      msg = data_[my_start];
      ++(*index);
    }
    MessageHeader *const header = MessageHeader::Get(msg);
    ++header->ref_count;
    if (kRefDebug) {
      printf("ref_inc_count: %p\n", msg);
    }
  }
  ReadCommonEnd(&read_data);
  return msg;
}

void *RawQueue::GetMessage() {
  MutexLocker locker(&pool_lock_);
  MessageHeader *header;
  if (pool_length_ - messages_used_ > 0) {
    header = pool_[messages_used_];
  } else {
    if (pool_length_ >= mem_length_) {
      LOG(FATAL, "overused pool of queue %p\n", this);
    }
    header = pool_[pool_length_] =
        static_cast<MessageHeader *>(shm_malloc(msg_length_));
    ++pool_length_;
  }
  void *msg = reinterpret_cast<uint8_t *>(header) + sizeof(MessageHeader);
  header->ref_count = 1;
  if (kRefDebug) {
    printf("%p ref alloc: %p\n", this, msg);
  }
  header->index = messages_used_;
  ++messages_used_;
  return msg;
}

}  // namespace aos
