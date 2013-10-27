#ifndef _SHARED_MEM_H_
#define _SHARED_MEM_H_

#include <stddef.h>
#include <unistd.h>
#include <time.h>

#include "aos/atom_code/ipc_lib/aos_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct aos_core *global_core;

// Where the shared memory segment starts in each process's address space.
// Has to be the same in all of them so that stuff in shared memory
// can have regular pointers to other stuff in shared memory.
#define SHM_START 0x20000000

typedef struct aos_queue_global_t {
  mutex alloc_lock;
  void *queue_list;  // an aos::Queue* declared in C code
} aos_queue_global;

typedef struct aos_shm_core_t {
  // clock_gettime(CLOCK_REALTIME, &identifier) gets called to identify
  // this shared memory area
  struct timespec identifier;
  // gets 0-initialized at the start (as part of shared memory) and
  // the owner sets as soon as it finishes setting stuff up
  mutex creation_condition;
  mutex msg_alloc_lock;
  void *msg_alloc;
  aos_queue_global queues;
} aos_shm_core;

enum aos_core_create {
  create,
  reference
};
struct aos_core {
  int owner;
  void *shared_mem;
  // How large the chunk of shared memory is.
  ptrdiff_t size;
  aos_shm_core *mem_struct;
};

void init_shared_mem_core(aos_shm_core *shm_core);

ptrdiff_t aos_core_get_mem_usage(void);

// Takes the specified memory address and uses it as the shared memory.
// address is the memory address, and size is the size of the memory.
// global_core needs to point to an instance of struct aos_core, and owner
// should be set correctly there.
// The owner should verify that the first sizeof(mutex) of data is set to 0
// before passing the memory to this function.
int aos_core_use_address_as_shared_mem(void *address, size_t size);

int aos_core_create_shared_mem(enum aos_core_create to_create);
int aos_core_free_shared_mem(void);

#ifdef __cplusplus
}
#endif

#endif
