#ifndef _AOS_CORE_LIB_H_
#define _AOS_CORE_LIB_H_

#include <stdint.h>

#include "aos/atom_code/ipc_lib/aos_sync.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

void *shm_malloc_aligned(size_t length, uint8_t alignment)
    __attribute__((alloc_size(1)));
static void *shm_malloc(size_t length) __attribute__((alloc_size(1)));
static inline void *shm_malloc(size_t length) {
  return shm_malloc_aligned(length, 0);
}

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif
