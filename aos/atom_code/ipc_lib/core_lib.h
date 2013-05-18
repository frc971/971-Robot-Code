#ifndef _AOS_CORE_LIB_H_
#define _AOS_CORE_LIB_H_

// defined in shared_mem.c
#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus
extern struct aos_core *global_core;
#ifdef __cplusplus
}
#endif  // __cplusplus

#include "aos_sync.h"
#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

struct aos_queue_list_t;
typedef struct aos_queue_hash_t {
	int alloc_flag;
	mutex alloc_lock;
	struct aos_queue_list_t *queue_list;
} aos_queue_hash;

typedef struct aos_shm_core_t {
  // clock_gettime(CLOCK_REALTIME, &identifier) gets called to identify
  // this shared memory area
  struct timespec identifier;
  // gets 0-initialized at the start (as part of shared memory) and
  // the owner sets as soon as it finishes setting stuff up
  mutex creation_condition;
  mutex msg_alloc_lock;
  void *msg_alloc;
  aos_queue_hash queues;
} aos_shm_core;

void init_shared_mem_core(aos_shm_core *shm_core);

void *shm_malloc_aligned(size_t length, uint8_t alignment);
static void *shm_malloc(size_t length);
static inline void *shm_malloc(size_t length) {
  return shm_malloc_aligned(length, 0);
}

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif
