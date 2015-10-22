#ifndef AOS_LINUX_CODE_IPC_LIB_SHARED_MEM_TYPES_H_
#define AOS_LINUX_CODE_IPC_LIB_SHARED_MEM_TYPES_H_

#include <stddef.h>

#include "aos/linux_code/ipc_lib/aos_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct aos_core *global_core __attribute__((weak));

// Where the shared memory segment starts in each process's address space.
// Has to be the same in all of them so that stuff in shared memory
// can have regular pointers to other stuff in shared memory.
#define SHM_START 0x20000000

// A structure that represents some kind of global pointer that everything
// shares.
typedef struct aos_global_pointer_t {
  struct aos_mutex lock;
  void *pointer;
} aos_global_pointer;

typedef struct aos_shm_core_t {
  // Gets 0-initialized at the start (as part of shared memory) and
  // the owner sets as soon as it finishes setting stuff up.
  aos_condition creation_condition;

  // An offset from CLOCK_REALTIME to times for all the code.
  // This is currently only set to non-zero by the log replay code.
  // There is no synchronization on this to avoid the overhead because it is
  // only updated with proper memory barriers when only a single task is
  // running.
  struct timespec time_offset;

  struct aos_mutex msg_alloc_lock;
  void *msg_alloc;

  // A pointer to the head of the linked list of queues.
  // pointer points to a ::aos::Queue.
  aos_global_pointer queues;
  // A pointer to the head of the linked list of queue message types.
  // pointer points to a ::aos::type_cache::ShmType.
  aos_global_pointer queue_types;
} aos_shm_core;

struct aos_core {
  // Non-0 if we "own" shared_mem and should shm_unlink(3) it when we're done.
  int owner;
  void *shared_mem;
  // How large the chunk of shared memory is.
  ptrdiff_t size;
  aos_shm_core *mem_struct;
  // For the owner to store the name of the file to unlink when closing.
  const char *shm_name;
};

#ifdef __cplusplus
}
#endif

#endif  // AOS_LINUX_CODE_IPC_LIB_SHARED_MEM_TYPES_H_
