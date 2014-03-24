#ifndef AOS_LINUX_CODE_IPC_LIB_QUEUE_H_
#define AOS_LINUX_CODE_IPC_LIB_QUEUE_H_

#include <assert.h>

#include "aos/linux_code/ipc_lib/shared_mem.h"
#include "aos/common/mutex.h"
#include "aos/common/condition.h"

// TODO(brians) add valgrind client requests to the queue and shared_mem_malloc
// code to make checking for leaks work better
// <http://www.valgrind.org/docs/manual/mc-manual.html#mc-manual.mempools>
// describes how

// Any pointers returned from these functions can be safely passed to other
// processes because they are all shared memory pointers.
// IMPORTANT: Any message pointer must be passed back in some way
// (FreeMessage and WriteMessage are common ones) or the
// application will leak shared memory.
// NOTE: Taking a message from ReadMessage and then passing it to WriteMessage
// might work, but it is not guaranteed to.

namespace aos {

// Queues are the primary way to use shared memory. Basic use consists of
// calling Queue::Fetch and then reading and/or writing messages.
// Queues (as the name suggests) are a FIFO stack of messages. Each combination
// of name and type signature will result in a different queue, which means
// that if you only recompile some code that uses differently sized messages,
// it will simply use a different queue than the old code.
class RawQueue {
 public:
  // Retrieves (and creates if necessary) a queue. Each combination of name and
  // signature refers to a completely independent queue.
  // length is how large each message will be
  // hash can differentiate multiple otherwise identical queues
  // queue_length is how many messages the queue will be able to hold
  // Will never return NULL.
  static RawQueue *Fetch(const char *name, size_t length, int hash,
                      int queue_length);
  // Same as above, except sets up the returned queue so that it will put
  // messages on *recycle when they are freed (after they have been released by
  // all other readers/writers and are not in the queue).
  // recycle_queue_length determines how many freed messages will be kept.
  // Other code can retrieve the 2 queues separately (the recycle queue will
  // have the same length and hash as the main one). However, any frees made
  // using a queue with only (name,length,hash,queue_length) before the
  // recycle queue has been associated with it will not go on to the recycle
  // queue.
  // NOTE: calling this function with the same (name,length,hash,queue_length)
  // but multiple recycle_queue_lengths will result in each freed message being
  // put onto an undefined one of the recycle queues.
  // Will never return NULL.
  static RawQueue *Fetch(const char *name, size_t length, int hash,
                      int queue_length,
                      int recycle_hash, int recycle_queue_length,
                      RawQueue **recycle);

  // Constants for passing to options arguments.
  // The non-conflicting ones can be combined with bitwise-or.

  // Doesn't update the currently read index (the read messages in the queue or
  // the index). This means the returned message (and any others skipped with
  // kFromEnd) will be left in the queue.
  // For reading only.
  static const int kPeek = 0x0001;
  // Reads the last message in the queue instead of just the next one.
  // NOTE: This removes all of the messages until the last one from the queue
  // (which means that nobody else will read them).
  // For reading only.
  static const int kFromEnd = 0x0002;
  // Causes reads to return NULL and writes to fail instead of waiting.
  // For reading and writing.
  static const int kNonBlock = 0x0004;
  // Causes things to block.
  // IMPORTANT: Has a value of 0 so that it is the default. This has to stay
  // this way.
  // For reading and writing.
  static const int kBlock = 0x0000;
  // Causes writes to overwrite the oldest message in the queue instead of
  // blocking.
  // For writing only.
  static const int kOverride = 0x0008;

  // Writes a message into the queue.
  // This function takes ownership of msg.
  // NOTE: msg must point to a valid message from this queue
  // Returns true on success. A return value of false means msg has already been
  // freed.
  bool WriteMessage(void *msg, int options);

  // Reads a message out of the queue.
  // The return value will have at least the length of this queue's worth of
  // valid data where it's pointing to.
  // The return value is const because other people might be viewing the same
  // messsage. Do not cast the const away!
  // IMPORTANT: The return value (if not NULL) must eventually be passed to
  // FreeMessage.
  const void *ReadMessage(int options);
  // The same as ReadMessage, except it will never return the
  // same message twice (when used with the same index argument). However,
  // may not return some messages that pass through the queue.
  // *index should start as 0. index does not have to be in shared memory, but
  // it can be.
  const void *ReadMessageIndex(int options, int *index);

  // Retrieves ("allocates") a message that can then be written to the queue.
  // NOTE: the return value will be completely uninitialized
  // The return value will have at least the length of this queue's worth of
  // valid memory where it's pointing to.
  // Returns NULL for error.
  // IMPORTANT: The return value (if not NULL) must eventually be passed to
  // FreeMessage or WriteMessage.
  void *GetMessage();

  // It is ok to call this method with a NULL msg.
  void FreeMessage(const void *msg) {
    if (msg != NULL) DecrementMessageReferenceCount(msg);
  }

  // UNSAFE! Returns the number of free messages we have. Only safe to use when
  // only 1 task is using this object (ie in tests).
  int FreeMessages() const;

 private:
  struct MessageHeader;

  // Adds 1 to the given index and handles wrapping correctly.
  int index_add1(int index);

  bool is_readable() { return data_end_ != data_start_; }
  bool is_writable() { return index_add1(data_end_) != data_start_; }

  // These next 4 allow finding the right one.
  const char *name_;
  size_t length_;
  int hash_;
  int queue_length_;
  // The next one in the linked list of queues.
  RawQueue *next_;

  RawQueue *recycle_;

  Mutex data_lock_;  // protects operations on data_ etc
  // Always gets broadcasted to because different readers might have different
  // ideas of what "readable" means (ie ones using separated indices).
  Condition readable_;
  Condition writable_;
  int data_length_;  // max length into data + 1
  int data_start_;  // is an index into data
  int data_end_;  // is an index into data
  int messages_;  // that have passed through
  void **data_;  // array of messages (with headers)

  size_t msg_length_;  // sizeof(each message) including the header
  // A pointer to the first in the linked list of free messages.
  MessageHeader *free_messages_;

  // Keeps track of if the queue was writable before a read so we can Signal() a
  // reader if we transition it.
  bool writable_start_;

  // Actually frees the given message.
  void DoFreeMessage(const void *msg);
  // Calls DoFreeMessage if appropriate.
  void DecrementMessageReferenceCount(const void *msg);
  // Only does the actual incrementing of the reference count.
  void IncrementMessageReferenceCount(const void *msg) const;

  // Must be called with data_lock_ locked.
  // *read_data will be initialized.
  // Returns with a readable message in data_ or false.
  bool ReadCommonStart(int options, int *index);
  // Deals with setting/unsetting readable_ and writable_.
  // Must be called after data_lock_ has been unlocked.
  // read_data should be the same thing that was passed in to ReadCommonStart.
  void ReadCommonEnd();
  // Returns the index of the last message.
  // Useful for reading with kPeek.
  int LastMessageIndex() const;

  // Gets called by Fetch when necessary (with placement new).
  RawQueue(const char *name, size_t length, int hash, int queue_length);
};

}  // namespace aos

#endif  // AOS_LINUX_CODE_IPC_LIB_QUEUE_H_
