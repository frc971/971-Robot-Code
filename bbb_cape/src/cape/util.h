#ifndef CAPE_UTIL_H_
#define CAPE_UTIL_H_

#define ALIAS_WEAK(f) __attribute__ ((weak, alias (#f)))

// Prevents the compiler from reordering memory operations around this.
static inline void compiler_memory_barrier(void) {
  __asm__ __volatile__("" ::: "memory");
}

#endif  // CAPE_UTIL_H_
