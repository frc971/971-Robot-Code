#ifndef _SHARED_MEM_H_
#define _SHARED_MEM_H_

#include <stddef.h>
#include <unistd.h>
#include <time.h>

#include "aos/linux_code/ipc_lib/shared_mem_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void init_shared_mem_core(aos_shm_core *shm_core);

ptrdiff_t aos_core_get_mem_usage(void);

// Takes the specified memory address and uses it as the shared memory.
// address is the memory address, and size is the size of the memory.
// global_core needs to point to an instance of struct aos_core, and owner
// should be set correctly there.
// The owner should verify that the first sizeof(mutex) of data is set to 0
// before passing the memory to this function.
void aos_core_use_address_as_shared_mem(void *address, size_t size);

// create is true to remove any existing shm to create a fresh one or false to
// fail if it does not already exist.
// lock is true to lock shared memory into RAM or false to not.
void aos_core_create_shared_mem(int create, int lock);
void aos_core_free_shared_mem(void);

// Returns whether or not the shared memory system is active.
int aos_core_is_init(void);

#ifdef __cplusplus
}
#endif

#endif
