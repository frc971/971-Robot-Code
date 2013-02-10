#ifndef AOS_IPC_LIB_QUEUE_INTERNAL_H_
#define AOS_IPC_LIB_QUEUE_INTERNAL_H_

#include "shared_mem.h"
#include "aos_sync.h"

// Should only be used by queue.c. Contains definitions of the structures
// it uses.

// The number of extra messages the pool associated with each queue will be able
// to hold (for readers who are slow about freeing them).
#define EXTRA_MESSAGES 20

typedef struct aos_msg_header_t {
	int ref_count;
	int index; // in the pool
} aos_msg_header;
static inline void header_swap(aos_msg_header *l, aos_msg_header *r) {
  aos_msg_header tmp;
  tmp.ref_count = l->ref_count;
  tmp.index = l->index;
  l->ref_count = r->ref_count;
  l->index = r->index;
  r->ref_count = tmp.ref_count;
  r->index = tmp.index;
}

struct aos_queue_list_t {
	char *name;
	aos_type_sig sig;
	aos_queue *queue;
	aos_queue_list *next;
};

typedef struct aos_ring_buf_t {
	mutex buff_lock; // the main lock protecting operations on this buffer
  // conditions
	mutex writable;
	mutex readable;
	int length; // max index into data + 1
	int start; // is an index into data
	int end; // is an index into data
	int msgs; // that have passed through
	void **data; // array of messages (w/ headers)
} aos_ring_buf;

typedef struct aos_msg_pool_t {
	mutex pool_lock;
	size_t msg_length;
	int mem_length; // the number of messages
	int used; // number of messages
	int length; // number of allocated messages
	void **pool; // array of messages
} aos_msg_pool;

struct aos_queue_t {
	aos_msg_pool pool;
	aos_ring_buf buf;
  aos_queue *recycle;
};

#endif
