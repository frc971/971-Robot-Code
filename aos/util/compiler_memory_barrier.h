#ifndef AOS_UTIL_COMPILER_MEMORY_BARRIER_H_
#define AOS_UTIL_COMPILER_MEMORY_BARRIER_H_

// Prevents the compiler from reordering memory operations around this.
// Using this function makes it clearer what you're doing and easier to be
// portable.
static inline void aos_compiler_memory_barrier(void) {
  __asm__ __volatile__("" ::: "memory");
}

#endif  // AOS_UTIL_COMPILER_MEMORY_BARRIER_H_
