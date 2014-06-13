#ifndef _AOS_CORE_LIB_H_
#define _AOS_CORE_LIB_H_

#include <stdint.h>

#include "aos/linux_code/ipc_lib/aos_sync.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// alloc_size was taken out of clang in r197866. It appears that it never
// actually did anything.
#if defined(__clang__)
#define attribute_alloc_size(n)
#else
#define attribute_alloc_size(n) __attribute__((alloc_size(n)))
#endif

void *shm_malloc_aligned(size_t length, uint8_t alignment)
    attribute_alloc_size(1);
static void *shm_malloc(size_t length) attribute_alloc_size(1);
static inline void *shm_malloc(size_t length) {
  return shm_malloc_aligned(length, 0);
}

#undef attribute_alloc_size

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif
