#include "aos/linux_code/ipc_lib/core_lib.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "aos/linux_code/ipc_lib/shared_mem_types.h"

static uint8_t aos_8max(uint8_t l, uint8_t r) {
  return (l > r) ? l : r;
}
void *shm_malloc_aligned(size_t length, uint8_t alignment) {
  // minimum alignments from
  // <http://software.intel.com/en-us/articles/data-alignment-when-migrating-to-64-bit-intel-architecture/>
  if (length <= 1) {
    alignment = aos_8max(alignment, 1);
  } else if (length <= 2) {
    alignment = aos_8max(alignment, 2);
  } else if (length <= 4) {
    alignment = aos_8max(alignment, 4);
  } else if (length <= 8) {
    alignment = aos_8max(alignment, 8);
  } else if (length <= 16) {
    alignment = aos_8max(alignment, 16);
  } else {
    alignment = aos_8max(alignment, (length >= 64) ? 64 : 16);
  }

  void *msg = NULL;
  aos_shm_core *shm_core = global_core->mem_struct;
  int result =
      mutex_grab(&shm_core->msg_alloc_lock);
#ifdef NDEBUG
  (void)result;
#else
  assert(result == 0);
#endif
  shm_core->msg_alloc = (uint8_t *)shm_core->msg_alloc - length;
  const uint8_t align_extra = (uintptr_t)shm_core->msg_alloc % alignment;
  shm_core->msg_alloc = (uint8_t *)shm_core->msg_alloc - align_extra;
  msg = shm_core->msg_alloc;
  if (msg <= global_core->shared_mem) {
    fprintf(stderr, "core_lib: RAN OUT OF SHARED MEMORY!!!----------------------------------------------------------\n");
    printf("if you didn't see the stderr output just then, you should have\n");
    abort();
  }
  //printf("alloc %p\n", msg);
  mutex_unlock(&shm_core->msg_alloc_lock);
  return msg;
}

