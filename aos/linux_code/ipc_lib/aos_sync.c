#include "aos/linux_code/ipc_lib/aos_sync.h"

#include <linux/futex.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <errno.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <inttypes.h>

#include "aos/common/logging/logging.h"

// TODO(brians): Inline these in the new PI version.
#define cmpxchg(ptr, o, n) __sync_val_compare_and_swap(ptr, o, n)
static inline uint32_t xchg(mutex *pointer, uint32_t value) {
  uint32_t result;
  __atomic_exchange(pointer, &value, &result, __ATOMIC_SEQ_CST);
  return result;
}

// this code is based on something that appears to be based on
//   <http://www.akkadia.org/drepper/futex.pdf>, which also has a lot of useful
//   information
// should probably use
// <http://lxr.linux.no/linux+v2.6.34/Documentation/robust-futexes.txt> once it
// becomes available
//   (sys_set_robust_list appears to be the function name)
// <http://locklessinc.com/articles/futex_cheat_sheet/> and
//   <http://locklessinc.com/articles/mutex_cv_futex/> are useful
// <http://lwn.net/Articles/360699/> has a nice overview of futexes in late 2009
//   (fairly recent compared to everything else...)
// can't use PRIVATE futex operations because they use the pid (or something) as
//   part of the hash
//
// Remember that EAGAIN and EWOUDBLOCK are the same! (ie if you get EAGAIN from
// FUTEX_WAIT, the docs call it EWOULDBLOCK...)
//
// Values for a mutex:
// 0 = unlocked
// 1 = locked, not contended
// 2 = locked, probably contended
// Values for a "futex":
// 0 = unset
// 1 = set

// These sys_futex_* functions are wrappers around syscall(SYS_futex). They each
// take a specific set of arguments for a given futex operation. They return the
// result or a negated errno value. -1..-4095 mean errors and not successful
// results, which is guaranteed by the kernel.
// They each have optimized versions for ARM EABI (the syscall interface is
// different for non-EABI ARM, so that is the right thing to test for) that
// don't go through syscall(2) or errno.

static inline int sys_futex_wait(mutex *addr1, int val1,
                                 const struct timespec *timeout) {
#ifdef __ARM_EABI__
  register mutex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_WAIT;
  register int val1_reg __asm__("r2") = val1;
  register const struct timespec *timeout_reg __asm__("r3") = timeout;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(val1_reg),
                     "r"(timeout_reg), "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, FUTEX_WAIT, val1, timeout);
  if (r == -1) return -errno;
  return r;
#endif
}

static inline int sys_futex_wake(mutex *addr1, int val1) {
#ifdef __ARM_EABI__
  register mutex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_WAKE;
  register int val1_reg __asm__("r2") = val1;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(val1_reg),
                     "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, FUTEX_WAKE, val1);
  if (r == -1) return -errno;
  return r;
#endif
}

static inline int sys_futex_requeue(mutex *addr1, int num_wake,
    int num_requeue, mutex *m) {
#ifdef __ARM_EABI__
  register mutex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_REQUEUE;
  register int num_wake_reg __asm__("r2") = num_wake;
  register int num_requeue_reg __asm__("r3") = num_requeue;
  register mutex *m_reg __asm__("r4") = m;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(num_wake_reg),
                     "r"(num_requeue_reg), "r"(m_reg), "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r =
      syscall(SYS_futex, addr1, FUTEX_REQUEUE, num_wake, num_requeue, m);
  if (r == -1) return -errno;
  return r;
#endif
}

static inline int mutex_get(mutex *m, uint8_t signals_fail, const
                            struct timespec *timeout) {
  int c;
  c = cmpxchg(m, 0, 1);
  if (!c) return 0;
  /* The lock is now contended */
  if (c == 1) c = xchg(m, 2);
  while (c) {
    /* Wait in the kernel */
    const int ret = sys_futex_wait(m, 2, timeout);
    if (ret != 0) {
      if (signals_fail && ret == -EINTR) {
        return 1;
      }
      if (timeout != NULL && ret == -ETIMEDOUT) {
        return 2;
      }
    }
    c = xchg(m, 2);
  }
  return 0;
}
int mutex_lock(mutex *m) {
  return mutex_get(m, 1, NULL);
}
int mutex_lock_timeout(mutex *m, const struct timespec *timeout) {
  return mutex_get(m, 1, timeout);
}
int mutex_grab(mutex *m) {
  return mutex_get(m, 0, NULL);
}

void mutex_unlock(mutex *m) {
  /* Unlock, and if not contended then exit. */
  switch (xchg(m, 0)) {
    case 0:
      LOG(FATAL, "multiple unlock of %p\n", m);
    case 1:
      break;
    case 2: {
      const int ret = sys_futex_wake(m, 1);
      if (ret < 0) {
        PELOG(FATAL, -ret, "waking 1 from %p failed", m);
      } else {
        break;
      }
    }
    default:
      LOG(FATAL, "got a garbage value from mutex %p\n", m);
  }
}
int mutex_trylock(mutex *m) {
  /* Try to take the lock, if is currently unlocked */
  unsigned c = cmpxchg(m, 0, 1);
  if (!c) return 0;
  return 1;
}

int futex_wait(mutex *m) {
  if (*m) {
    return 0;
  }
  const int ret = sys_futex_wait(m, 0, NULL);
  if (ret != 0) {
    if (ret == -EINTR) {
      return 1;
    } else if (ret != -EWOULDBLOCK) {
      errno = -ret;
      return -1;
    }
  }
  return 0;
}
int futex_set_value(mutex *m, mutex value) {
  xchg(m, value);
  const int r = sys_futex_wake(m, INT_MAX - 4096);
  if (__builtin_expect((unsigned int)r > (unsigned int)-4096, 0)) {
    errno = -r;
    return -1;
  } else {
    return r;
  }
}
int futex_set(mutex *m) {
  return futex_set_value(m, 1);
}
int futex_unset(mutex *m) {
  return !xchg(m, 0);
}

void condition_wait(mutex *c, mutex *m) {
  const mutex wait_start = *c;

  mutex_unlock(m);

  while (1) {
    // Wait in the kernel iff the value of it doesn't change (ie somebody else
    // does a wake) from before we unlocked the mutex.
    const int ret = sys_futex_wait(c, wait_start, NULL);
    if (ret != 0) {
      // If it failed for some reason other than somebody else doing a wake
      // before we actually made it to sleep.
      if (__builtin_expect(*c == wait_start, 0)) {
        // Try again if it was because of a signal.
        if (ret == -EINTR) continue;
        PELOG(FATAL, -ret, "FUTEX_WAIT(%p, %" PRIu32 ", NULL, NULL, 0) failed",
              c, wait_start);
      }
    }
    // Relock the mutex now that we're done waiting.
    // Simplified mutex_lock that always leaves it
    // contended in case anybody else got requeued.
    // If we got requeued above, this will just succeed the first time because
    // the person waking us from the above wait (changed to be on the mutex
    // instead of the condition) will have just set it to 0.
    while (xchg(m, 2) != 0) {
      const int ret = sys_futex_wait(m, 2, NULL);
      if (ret != 0) {
        // Try again if it was because of a signal or somebody else unlocked it
        // before we went to sleep.
        if (ret == -EINTR || ret == -EWOULDBLOCK) continue;
        PELOG(FATAL, -ret, "FUTEX_WAIT(%p, 2, NULL, NULL, 0) failed", m);
      }
    }
    return;
  }
}

void condition_signal(mutex *c) {
  // This will cause anybody else who is in between unlocking the mutex and
  // going to sleep in the kernel to not go to sleep and return immediately
  // instead.
  __sync_fetch_and_add(c, 1);
  // Wake at most 1 person who is waiting in the kernel.
  const int ret = sys_futex_wake(c, 1);
  if (ret < 0) {
    PELOG(FATAL, -ret, "FUTEX_WAKE(%p, 1, NULL, NULL, 0) failed", c);
  }
}

void condition_broadcast(mutex *c, mutex *m) {
  __sync_fetch_and_add(c, 1);
  // Wake at most 1 waiter and requeue the rest.
  // Everybody else is going to have to wait for the 1st person to take the
  // mutex anyways.
  const int ret = sys_futex_requeue(c, 1, INT_MAX, m);
  if (ret < 0) {
    PELOG(FATAL, -ret, "FUTEX_REQUEUE(%p, 1, INT_MAX, %p, 0) failed", c, m);
  }
}
