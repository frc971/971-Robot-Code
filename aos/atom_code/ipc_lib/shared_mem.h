#ifndef _SHARED_MEM_H_
#define _SHARED_MEM_H_

#include "core_lib.h"
#include <stddef.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

// Where the shared memory segment starts in each process's address space.
// Has to be the same in all of them so that stuff in shared memory
// can have regular pointers to other stuff in shared memory.
#define SHM_START 0x20000000

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
