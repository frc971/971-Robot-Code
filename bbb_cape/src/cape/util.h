#ifndef CAPE_UTIL_H_
#define CAPE_UTIL_H_

#define ALIAS_WEAK(f) __attribute__ ((weak, alias (#f)))

// MSG has to be separated_with_spaces.
#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(!!(COND))*2-1]

// Prevents the compiler from reordering memory operations around this.
static inline void compiler_memory_barrier(void) {
  __asm__ __volatile__("" ::: "memory");
}

#endif  // CAPE_UTIL_H_
