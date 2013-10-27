#ifndef __ASM_CMPXCHG_H
#define __ASM_CMPXCHG_H

#include <stdint.h>

//TODO implement xchg using gcc's atomic builtins (http://gcc.gnu.org/onlinedocs/gcc-4.1.1/gcc/Atomic-Builtins.html)
//or maybe http://gcc.gnu.org/onlinedocs/gcc/_005f_005fatomic-Builtins.html
//__atomic_fetch_sub looks promising

#define cmpxchg(ptr, o, n) __sync_val_compare_and_swap(ptr, o, n)
/*#define xchg(ptr, n) ({typeof(*ptr) r; \
    do{ \
      r = *ptr; \
    }while(!__sync_bool_compare_and_swap(ptr, r, n)); \
    r; \
})*/

#  define LOCK "lock;"
#  define LOCK_PREFIX "lock;"

#define xchg(ptr,v) ((__typeof__(*(ptr)))__xchg((unsigned long)(v),(ptr),sizeof(*(ptr))))

#define __xg(x) ((volatile long long *)(x))

/*static inline void set_64bit(volatile unsigned long *ptr, unsigned long val)
{
  *ptr = val;
}

#define _set_64bit set_64bit*/

/*
 * Note: no "lock" prefix even on SMP: xchg always implies lock anyway
 * Note 2: xchg has side effect, so that attribute volatile is necessary,
 *    but generally the primitive is invalid, *ptr is output argument. --ANK
 */
static inline unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
  switch (size) {
    case 1:
      __asm__ __volatile__("xchgb %b0,%1"
          :"=q" (x)
          :"m" (*__xg(ptr)), "0" (x)
          :"memory");
      break;
    case 2:
      __asm__ __volatile__("xchgw %w0,%1"
          :"=r" (x)
          :"m" (*__xg(ptr)), "0" (x)
          :"memory");
      break;
    case 4:
      __asm__ __volatile__("xchgl %k0,%1"
          :"=r" (x)
          :"m" (*__xg(ptr)), "0" (x)
          :"memory");
      break;
    case 8:
      __asm__ __volatile__("xchg %0,%1"
          :"=r" (x)
          :"m" (*__xg(ptr)), "0" (x)
          :"memory");
      break;
  }
  return x;
}

/*
 * Atomic compare and exchange.  Compare OLD with MEM, if identical,
 * store NEW in MEM.  Return the initial value in MEM.  Success is
 * indicated by comparing RETURN with OLD.
 */

#if 0

#define __HAVE_ARCH_CMPXCHG 1

static inline unsigned long __cmpxchg(volatile void *ptr, unsigned long old,
    unsigned long new, int size)
{
  int32_t prev;
  switch (size) {
    case 1:
      __asm__ __volatile__(LOCK_PREFIX "cmpxchgb %b1,%2"
            : "=a"(prev)
            : "q"(new), "m"(*__xg(ptr)), "0"(old)
            : "memory");
      return prev;
    case 2:
      __asm__ __volatile__(LOCK_PREFIX "cmpxchgw %w1,%2"
            : "=a"(prev)
            : "r"(new), "m"(*__xg(ptr)), "0"(old)
            : "memory");
      return prev;
    case 4:
      __asm__ __volatile__(LOCK_PREFIX "cmpxchgl %k1,%2"
            : "=a"(prev)
            : "r"(new), "m"(*__xg(ptr)), "0"(old)
            : "memory");
      return prev;
    case 8:
      __asm__ __volatile__("lock; cmpxchg %1,%2"
            : "=a"(prev)
            : "q"(new), "m"(*__xg(ptr)), "0"(old)
            : "memory");
      return prev;
  }
  return old;
}

/*
static inline unsigned long __cmpxchg_local(volatile void *ptr,
      unsigned long old, unsigned long new, int size)
{
  unsigned long prev;
  switch (size) {
  case 1:
    __asm__ __volatile__("cmpxchgb %b1,%2"
             : "=a"(prev)
             : "q"(new), "m"(*__xg(ptr)), "0"(old)
             : "memory");
    return prev;
  case 2:
    __asm__ __volatile__("cmpxchgw %w1,%2"
             : "=a"(prev)
             : "r"(new), "m"(*__xg(ptr)), "0"(old)
             : "memory");
    return prev;
  case 4:
    __asm__ __volatile__("cmpxchgl %k1,%2"
             : "=a"(prev)
             : "r"(new), "m"(*__xg(ptr)), "0"(old)
             : "memory");
    return prev;
  case 8:
    __asm__ __volatile__("cmpxchgq %1,%2"
             : "=a"(prev)
             : "r"(new), "m"(*__xg(ptr)), "0"(old)
             : "memory");
    return prev;
  }
  return old;
}*/

#define cmpxchg(ptr,o,n)\
  ((__typeof__(*(ptr)))__cmpxchg((ptr),(unsigned long)(o),\
          (unsigned long)(n),sizeof(*(ptr))))
/*#define cmpxchg_local(ptr,o,n)\
  ((__typeof__(*(ptr)))__cmpxchg((ptr),(unsigned long)(o),\
          (unsigned long)(n),sizeof(*(ptr))))*/
#endif

#endif
