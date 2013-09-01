#include "aos/common/queue.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <memory>

#include "aos/common/logging/logging.h"
#include "aos/common/type_traits.h"

namespace aos {

namespace {

static_assert(shm_ok<Queue>::value, "Queue instances go into shared memory");

const bool kReadDebug = false;
const bool kWriteDebug = false;
const bool kRefDebug = false;
const bool kFetchDebug = false;

// The number of extra messages the pool associated with each queue will be able
// to hold (for readers who are slow about freeing them).
const int kExtraMessages = 20;

}  // namespace

struct Queue::MessageHeader {
  int ref_count;
  int index;  // in pool_
  static MessageHeader *Get(const void *msg) {
    return reinterpret_cast<MessageHeader *>(
        static_cast<uint8_t *>(const_cast<void *>(msg)) -
        sizeof(MessageHeader));
  }
  void Swap(MessageHeader *other) {
    MessageHeader temp;
    memcpy(&temp, other, sizeof(temp));
    memcpy(other, this, sizeof(*other));
    memcpy(this, &temp, sizeof(*this));
  }
};
static_assert(shm_ok<Queue::MessageHeader>::value, "the whole point"
              " is to stick it in shared memory");

// TODO(brians) maybe do this with atomic integer instructions so it doesn't
//   have to lock/unlock pool_lock_
void Queue::DecrementMessageReferenceCount(const void *msg) {
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

Queue::Queue(const char *name, size_t length, int hash, int queue_length) {
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
Queue *Queue::Fetch(const char *name, size_t length, int hash,
                    int queue_length) {
  if (kFetchDebug) {
    printf("fetching queue %s\n", name);
  }
  if (mutex_lock(&global_core->mem_struct->queues.alloc_lock) != 0) {
    return NULL;
  }
  Queue *current = static_cast<Queue *>(
      global_core->mem_struct->queues.queue_list);
  Queue *last = NULL;
  while (current != NULL) {
    // if we found a matching queue
    if (strcmp(current->name_, name) == 0 && current->length_ == length &&
        current->hash_ == hash && current->queue_length_ == queue_length) {
      mutex_unlock(&global_core->mem_struct->queues.alloc_lock);
      return current;
    } else {
      if (kFetchDebug) {
        printf("rejected queue %s strcmp=%d target=%s\n", current->name_,
               strcmp(current->name_, name), name);
      }
    }
    current = current->next_;
  }

  void *temp = shm_malloc(sizeof(Queue));
  current = new (temp) Queue(name, length, hash, queue_length);
  if (last == NULL) {  // if we don't have one to tack the new one on to
    global_core->mem_struct->queues.queue_list = current;
  } else {
    last->next_ = current;
  }

  mutex_unlock(&global_core->mem_struct->queues.alloc_lock);
  return current;
}
Queue *Queue::Fetch(const char *name, size_t length, int hash,
                    int queue_length,
                    int recycle_hash, int recycle_length, Queue **recycle) {
  Queue *r = Fetch(name, length, hash, queue_length);
  r->recycle_ = Fetch(name, length, recycle_hash, recycle_length);
  if (r == r->recycle_) {
    fprintf(stderr, "queue: r->recycle_(=%p) == r(=%p)\n", r->recycle_, r);
    printf("see stderr\n");
    abort();
  }
  *recycle = r->recycle_;
  return r;
}

void Queue::DoFreeMessage(const void *msg) {
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
      // also switch the messages between the pools
      pool_[header->index] = new_header;
      {
        MutexLocker locker(&recycle_->pool_lock_);
        recycle_->pool_[new_header->index] = header;
        // swap the information in both headers
        header->Swap(new_header);
        // don't unlock the other pool until all of its messages are valid
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

  // where the one we're freeing was
  int index = header->index;
  header->index = -1;
  if (index != messages_used_) {  // if we're not freeing the one on the end
    // put the last one where the one we're freeing was
    header = pool_[index] = pool_[messages_used_];
    // put the one we're freeing at the end
    pool_[messages_used_] = MessageHeader::Get(msg);
    // update the former last one's index
    header->index = index;
  }
}

bool Queue::WriteMessage(void *msg, int options) {
  if (kWriteDebug) {
    printf("queue: %p->WriteMessage(%p, %d)\n", this, msg, options);
  }
  if (msg == NULL || msg < reinterpret_cast<void *>(global_core->mem_struct) ||
      msg > static_cast<void *>((
              reinterpret_cast<uintptr_t>(global_core->mem_struct) +
              global_core->size))) {
    fprintf(stderr, "queue: attempt to write bad message %p to %p. aborting\n",
            msg, this);
    printf("see stderr\n");
    abort();
  }
  {
    MutexLocker locker(&data_lock_);
    int new_end = (data_end_ + 1) % data_length_;
    while (new_end == data_start_) {
      if (options & kNonBlock) {
        if (kWriteDebug) {
          printf("queue: not blocking on %p. returning -1\n", this);
        }
        return false;
      } else if (options & kOverride) {
        if (kWriteDebug) {
          printf("queue: overriding on %p\n", this);
        }
        // avoid leaking the message that we're going to overwrite
        DecrementMessageReferenceCount(data_[data_start_]);
        data_start_ = (data_start_ + 1) % data_length_;
      } else {  // kBlock
        if (kWriteDebug) {
          printf("queue: going to wait for writable_ of %p\n", this);
        }
        writable_.Wait(&data_lock_);
      }
      new_end = (data_end_ + 1) % data_length_;
    }
    data_[data_end_] = msg;
    ++messages_;
    data_end_ = new_end;
  }
  if (kWriteDebug) {
    printf("queue: setting readable of %p\n", this);
  }
  readable_.Signal();
  if (kWriteDebug) {
    printf("queue: write returning true on queue %p\n", this);
  }
  return true;
}

void Queue::ReadCommonEnd(bool read) {
  if (read) {
    writable_.Signal();
  }
}
bool Queue::ReadCommonStart(int options, int *index) {
  while (data_start_ == data_end_ || ((index != NULL) && messages_ <= *index)) {
    if (options & kNonBlock) {
      if (kReadDebug) {
        printf("queue: not going to block waiting on %p\n", this);
      }
      return false;
    } else {  // kBlock
      if (kReadDebug) {
        printf("queue: going to wait for readable of %p\n", this);
      }
      data_lock_.Unlock();
      // wait for a message to become readable
      readable_.Wait();
      if (kReadDebug) {
        printf("queue: done waiting for readable of %p\n", this);
      }
      data_lock_.Lock();
    }
  }
  if (kReadDebug) {
    printf("queue: %p->read start=%d end=%d\n", this, data_start_, data_end_);
  }
  return true;
}
void *Queue::ReadPeek(int options, int start) {
  void *ret;
  if (options & kFromEnd) {
    int pos = data_end_ - 1;
    if (pos < 0) {  // if it needs to wrap
      pos = data_length_ - 1;
    }
    if (kReadDebug) {
      printf("queue: reading from line %d: %d\n", __LINE__, pos);
    }
    ret = data_[pos];
  } else {
    if (kReadDebug) {
      printf("queue: reading from line %d: %d\n", __LINE__, start);
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
const void *Queue::ReadMessage(int options) {
  if (kReadDebug) {
    printf("queue: %p->ReadMessage(%d)\n", this, options);
  }
  void *msg = NULL;
  MutexLocker locker(&data_lock_);
  if (!ReadCommonStart(options, NULL)) {
    if (kReadDebug) {
      printf("queue: common returned false for %p\n", this);
    }
    return NULL;
  }
  if (options & kPeek) {
    msg = ReadPeek(options, data_start_);
  } else {
    if (options & kFromEnd) {
      while (1) {
        if (kReadDebug) {
          printf("queue: start of c2 of %p\n", this);
        }
        // This loop pulls each message out of the buffer.
        const int pos = data_start_;
        data_start_ = (data_start_ + 1) % data_length_;
        // if this is the last one
        if (data_start_ == data_end_) {
          if (kReadDebug) {
            printf("queue: reading from c2: %d\n", pos);
          }
          msg = data_[pos];
          break;
        }
        // it's not going to be in the queue any more
        DecrementMessageReferenceCount(data_[pos]);
      }
    } else {
      if (kReadDebug) {
        printf("queue: reading from d2: %d\n", data_start_);
      }
      msg = data_[data_start_];
      data_start_ = (data_start_ + 1) % data_length_;
    }
  }
  ReadCommonEnd(!(options & kPeek));
  if (kReadDebug) {
    printf("queue: read returning %p\n", msg);
  }
  return msg;
}
const void *Queue::ReadMessageIndex(int options, int *index) {
  if (kReadDebug) {
    printf("queue: %p->ReadMessageIndex(%d, %p(*=%d))\n",
           this, options, index, *index);
  }
  void *msg = NULL;
  {
    MutexLocker locker(&data_lock_);
    if (!ReadCommonStart(options, index)) {
      if (kReadDebug) {
        printf("queue: common returned false for %p\n", this);
      }
      return NULL;
    }
    // TODO(parker): Handle integer wrap on the index.
    const int offset = messages_ - *index;
    int my_start = data_end_ - offset;
    if (offset >= data_length_) {  // if we're behind the available messages
      // catch index up to the last available message
      *index += data_start_ - my_start;
      // and that's the one we're going to read
      my_start = data_start_;
    }
    if (my_start < 0) {  // if we want to read off the end of the buffer
      // unwrap where we're going to read from
      my_start += data_length_;
    }
    if (options & kPeek) {
      msg = ReadPeek(options, my_start);
    } else {
      if (options & kFromEnd) {
        if (kReadDebug) {
          printf("queue: start of c1 of %p\n", this);
        }
        int pos = data_end_ - 1;
        if (pos < 0) {  // if it wrapped
          pos = data_length_ - 1;  // unwrap it
        }
        if (kReadDebug) {
          printf("queue: reading from c1: %d\n", pos);
        }
        msg = data_[pos];
        *index = messages_;
      } else {
        if (kReadDebug) {
          printf("queue: reading from d1: %d\n", my_start);
        }
        msg = data_[my_start];
        ++(*index);
      }
      MessageHeader *const header = MessageHeader::Get(msg);
      ++header->ref_count;
      if (kRefDebug) {
        printf("ref_inc_count: %p\n", msg);
      }
    }
  }
  // this function never consumes one off the queue
  ReadCommonEnd(false);
  return msg;
}

void *Queue::GetMessage() {
  MutexLocker locker(&pool_lock_);
  MessageHeader *header;
  if (pool_length_ - messages_used_ > 0) {
    header = pool_[messages_used_];
  } else {
    if (pool_length_ >= mem_length_) {
      LOG(FATAL, "overused pool %p from queue %p\n", pool, queue);
    }
    header = pool_[pool_length_] =
        static_cast<MessageHeader *>(shm_malloc(msg_length_));
    ++pool_length_;
  }
  void *msg = reinterpret_cast<uint8_t *>(header) + sizeof(MessageHeader);
  header->ref_count = 1;
  if (kRefDebug) {
    printf("ref alloc: %p\n", msg);
  }
  header->index = messages_used_;
  ++messages_used_;
  return msg;
}

}  // namespace aos
