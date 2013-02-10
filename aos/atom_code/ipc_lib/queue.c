#include "aos/atom_code/ipc_lib/queue.h"
#include "aos/atom_code/ipc_lib/queue_internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#define READ_DEBUG 0
#define WRITE_DEBUG 0
#define REF_DEBUG 0

static inline aos_msg_header *get_header(void *msg) {
	return (aos_msg_header *)((uint8_t *)msg - sizeof(aos_msg_header));
}
static inline aos_queue *aos_core_alloc_queue() {
	return shm_malloc_aligned(sizeof(aos_queue), sizeof(int));
}
static inline void *aos_alloc_msg(aos_msg_pool *pool) {
	return shm_malloc(pool->msg_length);
}

// actually free the given message
static inline int aos_free_msg(aos_msg_pool *pool, void *msg, aos_queue *queue) {
#if REF_DEBUG
	if (pool->pool_lock == 0) {
		//LOG(WARNING, "unprotected\n");
	}
#endif
	aos_msg_header *header = get_header(msg);
	if (pool->pool[header->index] != header) { // if something's messed up
		fprintf(stderr, "queue: something is very very wrong with queue %p."
				" pool->pool(=%p)[header->index(=%d)] != header(=%p)\n",
				queue, pool->pool, header->index, header);
		printf("queue: see stderr\n");
		abort();
	}
#if REF_DEBUG
	printf("ref_free_count: %p\n", msg);
#endif
	--pool->used;

	if (queue->recycle != NULL) {
		void *const new_msg = aos_queue_get_msg(queue->recycle);
		if (new_msg == NULL) {
			fprintf(stderr, "queue: couldn't get a message"
					" for recycle queue %p\n", queue->recycle);
		} else {
			// Take a message from recycle_queue and switch its
			// header with the one being freed, which effectively
			// switches which queue each message belongs to.
			aos_msg_header *const new_header = get_header(new_msg);
			// also switch the messages between the pools
			pool->pool[header->index] = new_header;
			if (mutex_lock(&queue->recycle->pool.pool_lock)) {
				return -1;
			}
			queue->recycle->pool.pool[new_header->index] = header;
			// swap the information in both headers
			header_swap(header, new_header);
			// don't unlock the other pool until all of its messages are valid
			mutex_unlock(&queue->recycle->pool.pool_lock);
			// use the header for new_msg which is now for this pool
			header = new_header;
			if (aos_queue_write_msg_free(queue->recycle,
						(void *)msg, OVERRIDE) != 0) {
				printf("queue: warning aos_queue_write_msg("
						"%p(=queue(=%p)->recycle), %p, OVERRIDE)"
						" failed\n",
						queue->recycle, queue, msg);
			}
			msg = new_msg;
		}
	}

	// where the one we're freeing was
	int index = header->index;
	header->index = -1;
	if (index != pool->used) { // if we're not freeing the one on the end
		// put the last one where the one we're freeing was
		header = pool->pool[index] = pool->pool[pool->used];
		// put the one we're freeing at the end
		pool->pool[pool->used] = get_header(msg);
		// update the former last one's index
		header->index = index;
	}
	return 0;
}
// TODO(brians) maybe do this with atomic integer instructions so it doesn't have to lock/unlock pool_lock
static inline int msg_ref_dec(void *msg, aos_msg_pool *pool, aos_queue *queue) {
	if (msg == NULL) {
		return 0;
	}

	int rv = 0;
	if (mutex_lock(&pool->pool_lock)) {
		return -1;
	}
	aos_msg_header *const header = get_header(msg);
	header->ref_count --;
	assert(header->ref_count >= 0);
#if REF_DEBUG
	printf("ref_dec_count: %p count=%d\n", msg, header->ref_count);
#endif
	if (header->ref_count == 0) {
		rv = aos_free_msg(pool, msg, queue);
	}
	mutex_unlock(&pool->pool_lock);
	return rv;
}

static inline int sigcmp(const aos_type_sig *sig1, const aos_type_sig *sig2) {
	if (sig1->length != sig2->length) {
		//LOG(DEBUG, "length mismatch 1=%d 2=%d\n", sig1->length, sig2->length);
		return 0;
	}
	if (sig1->queue_length != sig2->queue_length) {
		//LOG(DEBUG, "queue_length mismatch 1=%d 2=%d\n", sig1->queue_length, sig2->queue_length);
		return 0;
	}
	if (sig1->hash != sig2->hash) {
		//LOG(DEBUG, "hash mismatch 1=%d 2=%d\n", sig1->hash, sig2->hash);
		return 0;
	}
	//LOG(DEBUG, "signature match\n");
	return 1;
}
static inline aos_queue *aos_create_queue(const aos_type_sig *sig) {
	aos_queue *const queue = aos_core_alloc_queue();
	aos_msg_pool *const pool = &queue->pool;
	pool->mem_length = sig->queue_length + EXTRA_MESSAGES;
	pool->length = 0;
	pool->used = 0;
	pool->msg_length = sig->length + sizeof(aos_msg_header);
	pool->pool = shm_malloc(sizeof(void *) * pool->mem_length);
	aos_ring_buf *const buf = &queue->buf;
	buf->length = sig->queue_length + 1;
	if (buf->length < 2) { // TODO(brians) when could this happen?
		buf->length = 2;
	}
	buf->data = shm_malloc(buf->length * sizeof(void *));
	buf->start = 0;
	buf->end = 0;
	buf->msgs = 0;
	buf->writable = 1;
	buf->readable = 0;
	buf->buff_lock = 0;
	pool->pool_lock = 0;
	queue->recycle = NULL;
	return queue;
}
aos_queue *aos_fetch_queue(const char *name, const aos_type_sig *sig) {
	//LOG(DEBUG, "Fetching the stupid queue: %s\n", name);
	mutex_grab(&global_core->mem_struct->queues.alloc_lock);
	aos_queue_list *list = global_core->mem_struct->queues.queue_list;
	aos_queue_list *last = NULL;
	while (list != NULL) {
		// if we found a matching queue
		if (strcmp(list->name, name) == 0 && sigcmp(&list->sig, sig)) {
			mutex_unlock(&global_core->mem_struct->queues.alloc_lock);
			return list->queue;
		} else {
			//LOG(DEBUG, "rejected queue %s strcmp=%d target=%s\n", (*list)->name, strcmp((*list)->name, name), name);
		}
		last = list;
		list = list->next;
	}
	list = shm_malloc(sizeof(aos_queue_list));
	if (last == NULL) {
		global_core->mem_struct->queues.queue_list = list;
	} else {
		last->next = list;
	}
	list->sig = *sig;
	const size_t name_size = strlen(name) + 1;
	list->name = shm_malloc(name_size);
	memcpy(list->name, name, name_size);
	//LOG(INFO, "creating queue{name=%s, sig.length=%zd, sig.hash=%d, sig.queue_length=%d}\n", name, sig->length, sig->hash, sig->queue_length);
	list->queue = aos_create_queue(sig);
	//LOG(DEBUG, "Made the stupid queue: %s happy?\n", name);
	list->next = NULL;
	mutex_unlock(&global_core->mem_struct->queues.alloc_lock);
	return list->queue;
}
aos_queue *aos_fetch_queue_recycle(const char *name, const aos_type_sig *sig,
		const aos_type_sig *recycle_sig, aos_queue **recycle) {
	if (sig->length != recycle_sig->length || sig->hash == recycle_sig->hash) {
		*recycle = NULL;
		return NULL;
	}
	aos_queue *const r = aos_fetch_queue(name, sig);
	r->recycle = aos_fetch_queue(name, recycle_sig);
	if (r == r->recycle) {
		fprintf(stderr, "queue: r->recycle(=%p) == r(=%p)\n", r->recycle, r);
		printf("see stderr\n");
		abort();
	}
	*recycle = r->recycle;
	return r;
}

int aos_queue_write_msg(aos_queue *queue, void *msg, int opts) {
#if WRITE_DEBUG
  printf("queue: write_msg(%p, %p, %d)\n", queue, msg, opts);
#endif
  int rv = 0;
  if (msg == NULL || msg < (void *)global_core->mem_struct ||
      msg > (void *)((intptr_t)global_core->mem_struct + global_core->size)) {
    fprintf(stderr, "queue: attempt to write bad message %p to %p. aborting\n",
            msg, queue);
    printf("see stderr\n");
    abort();
  }
  aos_ring_buf *const buf = &queue->buf;
  if (mutex_lock(&buf->buff_lock)) {
#if WRITE_DEBUG
    printf("queue: locking buff_lock of %p failed\n", buf);
#endif
    return -1;
  }
  int new_end = (buf->end + 1) % buf->length;
  while (new_end == buf->start) {
    if (opts & NON_BLOCK) {
#if WRITE_DEBUG
      printf("queue: not blocking on %p. returning -1\n", queue);
#endif
      mutex_unlock(&buf->buff_lock);
      return -1;
    } else if (opts & OVERRIDE) {
#if WRITE_DEBUG
      printf("queue: overriding on %p\n", queue);
#endif
      // avoid leaking the message that we're going to overwrite
      msg_ref_dec(buf->data[buf->start], &queue->pool, queue);
      buf->start = (buf->start + 1) % buf->length;
    } else { // BLOCK
      mutex_unlock(&buf->buff_lock);
#if WRITE_DEBUG
      printf("queue: going to wait for writable(=%p) of %p\n",
          &buf->writable, queue);
#endif
      if (condition_wait(&buf->writable)) {
#if WRITE_DEBUG
        printf("queue: waiting for writable(=%p) of %p failed\n",
            &buf->writable, queue);
#endif
        return -1;
      }
#if WRITE_DEBUG
      printf("queue: going to re-lock buff_lock of %p to write\n", queue);
#endif
      if (mutex_lock(&buf->buff_lock)) {
#if WRITE_DEBUG
        printf("queue: error locking buff_lock of %p\n", queue);
#endif
        return -1;
      }
    }
    new_end = (buf->end + 1) % buf->length;
  }
  buf->data[buf->end] = msg;
  ++buf->msgs;
  buf->end = new_end;
  mutex_unlock(&buf->buff_lock);
#if WRITE_DEBUG
  printf("queue: setting readable(=%p) of %p\n", &buf->readable, queue);
#endif
  condition_set(&buf->readable);
  if (((buf->end + 1) % buf->length) == buf->start) { // if it's now full
    condition_unset(&buf->writable);
  }
#if WRITE_DEBUG
  printf("queue: write returning %d on queue %p\n", rv, queue);
#endif
  return rv;
}

int aos_queue_free_msg(aos_queue *queue, const void *msg) {
	// TODO(brians) get rid of this
	void *msg_temp;
	memcpy(&msg_temp, &msg, sizeof(msg_temp));
  	return msg_ref_dec(msg_temp, &queue->pool, queue);
}
// Deals with setting/unsetting readable and writable.
// Should be called after buff_lock has been unlocked.
// read is whether or not this read call read one off the queue
static inline void aos_read_msg_common_end(aos_ring_buf *const buf, int read) {
	if (read) {
		condition_set(&buf->writable);
		if (buf->start == buf->end) {
			condition_unset(&buf->readable);
		}
	}
}
// Returns with buff_lock locked and a readable message in buf.
// Returns -1 for error (if it returns -1, buff_lock will be unlocked).
static inline int aos_queue_read_msg_common(int opts, aos_ring_buf *const buf,
		aos_queue *const queue, int *index) {
#if !READ_DEBUG
	(void)queue;
#endif
	if (mutex_lock(&buf->buff_lock)) {
#if READ_DEBUG
		printf("queue: couldn't lock buff_lock of %p\n", queue);
#endif
		return -1;
	}
	while (buf->start == buf->end || ((index != NULL) && buf->msgs <= *index)) {
		mutex_unlock(&buf->buff_lock);
		if (opts & NON_BLOCK) {
#if READ_DEBUG
			printf("queue: not going to block waiting on %p\n", queue);
#endif
			return -1;
		} else { // BLOCK
#if READ_DEBUG
			printf("queue: going to wait for readable(=%p) of %p\n",
					&buf->readable, queue);
#endif
			// wait for a message to become readable
			if ((index == NULL) ? condition_wait(&buf->readable) :
					condition_wait_force(&buf->readable)) {
#if READ_DEBUG
				printf("queue: waiting for readable(=%p) of %p failed\n",
						&buf->readable, queue);
#endif
				return -1;
			}
		}
#if READ_DEBUG
		printf("queue: going to re-lock buff_lock of %p to read\n", queue);
#endif
		if (mutex_lock(&buf->buff_lock)) {
#if READ_DEBUG
			printf("couldn't re-lock buff_lock of %p\n", queue);
#endif
			return -1;
		}
	}
#if READ_DEBUG
	printf("queue: read start=%d end=%d from %p\n", buf->start, buf->end, queue);
#endif
	return 0;
}
// handles reading with PEEK
static inline void *read_msg_peek(aos_ring_buf *const buf, int opts, int start) {
	void *ret;
	if (opts & FROM_END) {
		int pos = buf->end - 1;
		if (pos < 0) { // if it needs to wrap
			pos = buf->length - 1;
		}
#if READ_DEBUG
		printf("queue: reading from line %d: %d\n", __LINE__, pos);
#endif
		ret = buf->data[pos];
	} else {
#if READ_DEBUG
		printf("queue: reading from line %d: %d\n", __LINE__, start);
#endif
		ret = buf->data[start];
	}
	aos_msg_header *const header = get_header(ret);
	header->ref_count ++;
#if REF_DEBUG
	printf("ref inc count: %p\n", ret);
#endif
	return ret;
}
const void *aos_queue_read_msg(aos_queue *queue, int opts) {
#if READ_DEBUG
	printf("queue: read_msg(%p, %d)\n", queue, opts);
#endif
	void *msg = NULL;
	aos_ring_buf *const buf = &queue->buf;
	if (aos_queue_read_msg_common(opts, buf, queue, NULL) == -1) {
#if READ_DEBUG
		printf("queue: common returned -1 for %p\n", queue);
#endif
		return NULL;
	}
	if (opts & PEEK) {
		msg = read_msg_peek(buf, opts, buf->start);
	} else {
		if (opts & FROM_END) {
			while (1) {
#if READ_DEBUG
				printf("queue: start of c2 of %p\n", queue);
#endif
				// This loop pulls each message out of the buffer.
				const int pos = buf->start;
				buf->start = (buf->start + 1) % buf->length;
				// if this is the last one
				if (buf->start == buf->end) {
#if READ_DEBUG
					printf("queue: reading from c2: %d\n", pos);
#endif
					msg = buf->data[pos];
					break;
				}
				// it's not going to be in the queue any more
				msg_ref_dec(buf->data[pos], &queue->pool, queue);
			}
		} else {
#if READ_DEBUG
			printf("queue: reading from d2: %d\n", buf->start);
#endif
			msg = buf->data[buf->start];
			buf->start = (buf->start + 1) % buf->length;
		}
	}
	mutex_unlock(&buf->buff_lock);
	aos_read_msg_common_end(buf, !(opts & PEEK));
#if READ_DEBUG
	printf("queue: read returning %p\n", msg);
#endif
	return msg;
}
const void *aos_queue_read_msg_index(aos_queue *queue, int opts, int *index) {
#if READ_DEBUG
	printf("queue: read_msg_index(%p, %d, %p(*=%d))\n", queue, opts, index, *index);
#endif
	void *msg = NULL;
	aos_ring_buf *const buf = &queue->buf;
	if (aos_queue_read_msg_common(opts, buf, queue, index) == -1) {
#if READ_DEBUG
		printf("queue: common returned -1\n");
#endif
		return NULL;
	}
        // TODO(parker): Handle integer wrap on the index.
	const int offset = buf->msgs - *index;
	int my_start = buf->end - offset;
	if (offset >= buf->length) { // if we're behind the available messages
		// catch index up to the last available message
		*index += buf->start - my_start;
		// and that's the one we're going to read
		my_start = buf->start;
	}
	if (my_start < 0) { // if we want to read off the end of the buffer
		// unwrap where we're going to read from
		my_start += buf->length;
	}
	if (opts & PEEK) {
		msg = read_msg_peek(buf, opts, my_start);
	} else {
		if (opts & FROM_END) {
#if READ_DEBUG
			printf("queue: start of c1 of %p\n", queue);
#endif
			int pos = buf->end - 1;
			if (pos < 0) { // if it wrapped
				pos = buf->length - 1; // unwrap it
			}
#if READ_DEBUG
			printf("queue: reading from c1: %d\n", pos);
#endif
			msg = buf->data[pos];
			*index = buf->msgs;
		} else {
#if READ_DEBUG
			printf("queue: reading from d1: %d\n", my_start);
#endif
			msg = buf->data[my_start];
			++(*index);
		}
		aos_msg_header *const header = get_header(msg);
		++header->ref_count;
#if REF_DEBUG
		printf("ref_inc_count: %p\n", msg);
#endif
	}
	mutex_unlock(&buf->buff_lock);
	// this function never consumes one off the queue
	aos_read_msg_common_end(buf, 0);
	return msg;
}
static inline void *aos_pool_get_msg(aos_msg_pool *pool) {
	if (mutex_lock(&pool->pool_lock)) {
		return NULL;
	}
	void *msg;
	if (pool->length - pool->used > 0) {
		msg = pool->pool[pool->used];
	} else {
		if (pool->length >= pool->mem_length) {
			//TODO(brians) log this if it isn't the log queue
			fprintf(stderr, "queue: overused_pool\n");
			msg = NULL;
			goto exit;
		}
		msg = pool->pool[pool->length] = aos_alloc_msg(pool);
		++pool->length;
	}
	aos_msg_header *const header = msg;
	msg = (uint8_t *)msg + sizeof(aos_msg_header);
	header->ref_count = 1;
#if REF_DEBUG
	printf("ref alloc: %p\n", msg);
#endif
	header->index = pool->used;
	++pool->used;
exit:
	mutex_unlock(&pool->pool_lock);
	return msg;
}
void *aos_queue_get_msg(aos_queue *queue) {
	return aos_pool_get_msg(&queue->pool);
}

