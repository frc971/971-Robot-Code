#ifndef AOS_IPC_LIB_QUEUE_H_
#define AOS_IPC_LIB_QUEUE_H_

#include "shared_mem.h"
#include "aos_sync.h"

// TODO(brians) add valgrind client requests to the queue and shared_mem_malloc
// code to make checking for leaks work better
// <http://www.valgrind.org/docs/manual/mc-manual.html#mc-manual.mempools>
// describes how

#ifdef __cplusplus
extern "C" {
#endif

// Queues are the primary way to use shared memory. Basic use consists of
// initializing an aos_type_sig and then calling aos_fetch_queue on it.
// This aos_queue* can then be used to get a message and write it or to read a
// message.
// Queues (as the name suggests) are a FIFO stack of messages. Each combination
// of name and aos_type_sig will result in a different queue, which means that
// if you only recompile some code that uses differently sized messages, it will
// simply use a different queue than the old code.
//
// Any pointers returned from these functions can be safely passed to other
// processes because they are all shared memory pointers.
// IMPORTANT: Any message pointer must be passed back in some way
// (aos_queue_free_msg and aos_queue_write_msg are common ones) or the
// application will leak shared memory.
// NOTE: Taking a message from read_msg and then passing it to write_msg might
// work, but it is not guaranteed to.

typedef struct aos_type_sig_t {
	size_t length; // sizeof(message)
	int hash; // can differentiate multiple otherwise identical queues
	int queue_length; // how many messages the queue can hold
} aos_type_sig;

// Structures that are opaque to users (defined in queue_internal.h).
typedef struct aos_queue_list_t aos_queue_list;
typedef struct aos_queue_t aos_queue;

// Retrieves (and creates if necessary) a queue. Each combination of name and
// signature refers to a completely independent queue.
aos_queue *aos_fetch_queue(const char *name, const aos_type_sig *sig);
// Same as above, except sets up the returned queue so that it will put messages
// on *recycle (retrieved with recycle_sig) when they are freed (after they have
// been released by all other readers/writers and are not in the queue).
// The length of recycle_sig determines how many freed messages will be kept.
// Other code can retrieve recycle_sig and sig separately. However, any frees
// made using aos_fetch_queue with only sig before the recycle queue has been
// associated with it will not go on to the recyce queue.
// Will return NULL for both queues if sig->length != recycle_sig->length or
// sig->hash == recycle_sig->hash (just to be safe).
// NOTE: calling this function with the same sig but multiple recycle_sig s
// will result in each freed message being put onto an undefined recycle_sig.
aos_queue *aos_fetch_queue_recycle(const char *name, const aos_type_sig *sig,
                                   const aos_type_sig *recycle_sig, aos_queue **recycle);

// Constants for passing to opts arguments.
// #defines so that c code can use queues
// The non-conflicting ones can be combined with bitwise-or.
// TODO(brians) prefix these?
//
// Causes the returned message to be left in the queue.
// For reading only.
#define PEEK      0x0001
// Reads the last message in the queue instead of just the next one.
// NOTE: This removes all of the messages until the last one from the queue
// (which means that nobody else will read them). However, PEEK means to not
// remove any from the queue, including the ones that are skipped.
// For reading only.
#define FROM_END  0x0002
// Causes reads to return NULL and writes to fail instead of waiting.
// For reading and writing.
#define NON_BLOCK 0x0004
// Causes things to block.
// IMPORTANT: #defined to 0 so that it is the default. This has to stay.
// For reading and writing.
#define BLOCK     0x0000
// Causes writes to overwrite the oldest message in the queue instead of
// blocking.
// For writing only.
#define OVERRIDE  0x0008

// Frees a message. Does nothing if msg is NULL.
int aos_queue_free_msg(aos_queue *queue, const void *msg);

// Writes a message into the queue.
// NOTE: msg must point to at least the length of this queue's worth of valid
// data to write
// IMPORTANT: if this returns -1, then the caller must do something with msg
// (like free it)
int aos_queue_write_msg(aos_queue *queue, void *msg, int opts);
// Exactly the same as aos_queue_write_msg, except it automatically frees the
// message if writing fails.
static inline int aos_queue_write_msg_free(aos_queue *queue, void *msg, int opts) {
  const int ret = aos_queue_write_msg(queue, msg, opts);
  if (ret != 0) {
    aos_queue_free_msg(queue, msg);
  }
  return ret;
}

// Reads a message out of the queue.
// The return value will have at least the length of this queue's worth of valid
// data where it's pointing to.
// The return value is const because other people might be viewing the same
// messsage. Do not cast the const away!
// IMPORTANT: The return value (if not NULL) must eventually be passed to
// aos_queue_free_msg.
const void *aos_queue_read_msg(aos_queue *buf, int opts);
// Exactly the same as aos_queue_read_msg, except it will never return the same
// message twice with the same index argument. However, it may not return some
// messages that pass through the queue.
// *index should start as 0. index does not have to be in shared memory, but it
// can be
const void *aos_queue_read_msg_index(aos_queue *queue, int opts, int *index);

// Retrieves ("allocates") a message that can then be written to the queue.
// NOTE: the return value will be completely uninitialized
// The return value will have at least the length of this queue's worth of valid
// data where it's pointing to.
// Returns NULL for error.
// IMPORTANT: The return value (if not NULL) must eventually be passed to
// aos_queue_free_msg.
void *aos_queue_get_msg(aos_queue *queue);

#ifdef __cplusplus
}
#endif

#endif

