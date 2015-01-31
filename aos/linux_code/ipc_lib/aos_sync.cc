#if !AOS_DEBUG
#define NDEBUG
#endif

#include "aos/linux_code/ipc_lib/aos_sync.h"

#include <linux/futex.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <errno.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <inttypes.h>
#include <sys/types.h>
#include <stddef.h>
#include <assert.h>
#include <pthread.h>
#include <sched.h>

#include <algorithm>

#include "aos/common/logging/logging.h"
#include "aos/linux_code/thread_local.h"
#include "aos/common/once.h"
#include "aos/common/macros.h"

// This code was originally based on <http://www.akkadia.org/drepper/futex.pdf>,
// but is has since evolved a lot. However, that still has useful information.
//
// Finding information about actually using futexes is really REALLY hard, so
//   here's a list of the stuff that I've used:
// futex(7) has a really high-level overview.
// <http://locklessinc.com/articles/futex_cheat_sheet/> describes some of the
//   operations in a bit more detail than most places.
// <http://locklessinc.com/articles/mutex_cv_futex/> is the basis of our
//   implementations (before PI).
// <http://lwn.net/Articles/360699/> has a nice overview of futexes in late 2009
//   (fairly recent compared to everything else...).
// <https://www.kernel.org/doc/Documentation/pi-futex.txt>,
//   <https://www.kernel.org/doc/Documentation/futex-requeue-pi.txt>,
//   <https://www.kernel.org/doc/Documentation/robust-futexes.txt>,
//   and <https://www.kernel.org/doc/Documentation/robust-futex-ABI.txt> are all
//   useful references.
// The kernel source (kernel/futex.c) has some useful comments about what the
//   various operations do (except figuring out which argument goes where in the
//   syscall is still confusing).
// futex(2) is basically useless except for describing the order of the
//   arguments (it only has high-level descriptions of what some of the
//   operations do, and some of them are wrong in Wheezy).
// glibc's nptl pthreads implementation is the intended user of most of these
//   things, so it is also a good place to look for examples. However, it is all
//   very hard to read because it supports ~20 different kinds of mutexes and
//   several variations of condition variables, and some of the pieces of code
//   are only written in assembly.
// set_robust_list(2) is wrong in Wheezy (it doesn't actually take a TID
//   argument).
//
// Can't use PRIVATE futex operations because they use the pid (or something) as
//   part of the hash.
//
// ThreadSanitizer understands how these mutexes etc work. It appears to be able
// to figure out the happens-before relationship from the __ATOMIC_SEQ_CST
// atomic primitives.
//
// Remember that EAGAIN and EWOUDBLOCK are the same! (ie if you get EAGAIN from
// FUTEX_WAIT, the docs call it EWOULDBLOCK...)

// Values for an aos_mutex.futex (kernel-mandated):
// 0 = unlocked
// TID = locked, not contended
// |FUTEX_WAITERS = there are waiters (aka contended)
//
// Values for an aos_futex being used directly:
// 0 = unset
// 1 = set
//
// The value of an aos_condition is just a generation counter.

namespace {

// These sys_futex_* functions are wrappers around syscall(SYS_futex). They each
// take a specific set of arguments for a given futex operation. They return the
// result or a negated errno value. -1..-4095 mean errors and not successful
// results, which is guaranteed by the kernel.
//
// They each have optimized versions for ARM EABI (the syscall interface is
// different for non-EABI ARM, so that is the right thing to test for) that
// don't go through syscall(2) or errno.
// These use register variables to get the values in the right registers to
// actually make the syscall.

// The actual macro that we key off of to use the inline versions or not.
#define ARM_EABI_INLINE_SYSCALL defined(__ARM_EABI__)

// Used for FUTEX_WAIT, FUTEX_LOCK_PI, and FUTEX_TRYLOCK_PI.
inline int sys_futex_wait(int op, aos_futex *addr1, int val1,
                          const struct timespec *timeout) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = op;
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
  const int r = syscall(SYS_futex, addr1, op, val1, timeout);
  if (r == -1) return -errno;
  return r;
#endif
}

inline int sys_futex_wake(aos_futex *addr1, int val1) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
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

inline int sys_futex_unlock_pi(aos_futex *addr1) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_UNLOCK_PI;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, FUTEX_UNLOCK_PI);
  if (r == -1) return -errno;
  return r;
#endif
}

// Returns true if it succeeds and false if it fails.
// This is the same as __sync_bool_compare_and_swap, except it fixes that being
// broken under tsan.
inline bool compare_and_swap(aos_futex *f, uint32_t before, uint32_t after) {
#ifdef AOS_SANITIZER_thread
  // TODO(brians): Figure out how exactly tsan breaks this and fix it + make
  // sure our workaround actually works.
  // This workaround is unsafe in the general case, but does not change the
  // effect in our specific case if the primitive works correctly (and seems to
  // still do the right thing even when tsan's version falsely reports failing).
  if (__atomic_load_n(f, __ATOMIC_SEQ_CST) == after) return false;
  if (__sync_bool_compare_and_swap(f, before, after)) return true;
  return __atomic_load_n(f, __ATOMIC_SEQ_CST) == after;
#else
  return __sync_bool_compare_and_swap(f, before, after);
#endif
}

pid_t do_get_tid() {
  pid_t r = syscall(SYS_gettid);
  assert(r > 0);
  return r;
}

// This gets called by functions before LOG(FATAL)ing with error messages that
// would be incorrect if the error was caused by a process forking without
// initialize_in_new_thread getting called in the fork.
void check_cached_tid(pid_t tid) {
  pid_t actual = do_get_tid();
  if (tid != actual) {
    LOG(FATAL,
        "task %jd forked into %jd without letting aos_sync know"
        " so we're not really sure what's going on\n",
        static_cast<intmax_t>(tid), static_cast<intmax_t>(actual));
  }
}

// Starts off at 0 in each new thread (because that's what it gets initialized
// to in most of them or it gets to reset to 0 after a fork by atfork_child()).
AOS_THREAD_LOCAL pid_t my_tid = 0;

// Gets called before the fork(2) wrapper function returns in the child.
void atfork_child() {
  // The next time get_tid() is called, it will set everything up again.
  my_tid = 0;
}

void *InstallAtforkHook() {
  if (pthread_atfork(NULL, NULL, atfork_child) != 0) {
    PLOG(FATAL, "pthread_atfork(NULL, NULL, %p) failed", atfork_child);
  }
  return nullptr;
}

// This gets called to set everything up in a new thread by get_tid().
void initialize_in_new_thread();

// Gets the current thread's TID and does all of the 1-time initialization the
// first time it's called in a given thread.
inline uint32_t get_tid() {
  if (__builtin_expect(my_tid == 0, 0)) {
    initialize_in_new_thread();
  }
  static_assert(sizeof(my_tid) <= sizeof(uint32_t), "pid_t is too big");
  return static_cast<uint32_t>(my_tid);
}

void initialize_in_new_thread() {
  // No synchronization necessary in most of this because it's all thread-local!

  my_tid = do_get_tid();

  static ::aos::Once<void> atfork_hook_installed(InstallAtforkHook);
  atfork_hook_installed.Get();
}

// Split out separately from mutex_get so condition_wait can call it too.
inline int mutex_do_get(aos_mutex *m, bool signals_fail,
                        const struct timespec *timeout) {
  const uint32_t tid = get_tid();

  while (true) {
    // If the atomic 0->TID transition fails.
    if (!compare_and_swap(&m->futex, 0, tid)) {
      // Wait in the kernel, which handles atomically ORing in FUTEX_WAITERS
      // before actually sleeping.
      const int ret = sys_futex_wait(FUTEX_LOCK_PI, &m->futex, 1, timeout);
      if (ret != 0) {
        if (timeout != NULL && ret == -ETIMEDOUT) {
          return 3;
        }
        if (__builtin_expect(ret == -EINTR, 1)) {
          if (signals_fail) {
            return 2;
          } else {
            continue;
          }
        }
        if (ret == -EDEADLK) {
          LOG(FATAL, "multiple lock of %p by %" PRId32 "\n", m, tid);
        }
        PELOG(FATAL, -ret, "FUTEX_LOCK_PI(%p(=%" PRIu32 "), 1, %p) failed",
              &m->futex, __atomic_load_n(&m->futex, __ATOMIC_SEQ_CST), timeout);
      } else {
        // The kernel already handled setting the value to our TID (ish).
        break;
      }
    } else {
      // Fastpath succeeded, so no need to call into the kernel.
      break;
    }
  }

  return 0;
}

// The common implementation for everything that wants to lock a mutex.
// If signals_fail is false, the function will try again if the wait syscall is
// interrupted by a signal.
// timeout can be NULL for no timeout.
inline int mutex_get(aos_mutex *m, bool signals_fail,
                     const struct timespec *timeout) {
  get_tid();
  const int r = mutex_do_get(m, signals_fail, timeout);
  return r;
}

// The common implementation for broadcast and signal.
// number_requeue is the number of waiters to requeue (probably INT_MAX or 0). 1
// will always be woken.
void condition_wake(aos_condition *c, aos_mutex * /*m*/, int number_requeue) {
  // Make it so that anybody just going to sleep won't.
  // This is where we might accidentally wake more than just 1 waiter with 1
  // signal():
  //   1 already sleeping will be woken but n might never actually make it to
  //     sleep in the kernel because of this.
  __atomic_add_fetch(c, 1, __ATOMIC_SEQ_CST);

  const int ret = sys_futex_wake(
      c, ::std::min(::std::max(number_requeue, 1), INT_MAX - 4096));
  if (__builtin_expect(
          static_cast<unsigned int>(ret) > static_cast<unsigned int>(-4096),
          0)) {
    PELOG(FATAL, -ret, "FUTEX_WAKE(%p, %d) failed", c, INT_MAX - 4096);
  }
}

}  // namespace

int mutex_lock(aos_mutex *m) {
  return mutex_get(m, true, NULL);
}
int mutex_lock_timeout(aos_mutex *m, const struct timespec *timeout) {
  return mutex_get(m, true, timeout);
}
int mutex_grab(aos_mutex *m) {
  return mutex_get(m, false, NULL);
}

void mutex_unlock(aos_mutex *m) {
  const uint32_t tid = get_tid();

  const uint32_t value = __atomic_load_n(&m->futex, __ATOMIC_SEQ_CST);
  if (__builtin_expect((value & FUTEX_TID_MASK) != tid, 0)) {
    check_cached_tid(tid);
    if ((value & FUTEX_TID_MASK) == 0) {
      LOG(FATAL, "multiple unlock of aos_mutex %p by %" PRId32 "\n", m, tid);
    } else {
      LOG(FATAL, "aos_mutex %p is locked by %" PRId32 ", not %" PRId32 "\n",
          m, value & FUTEX_TID_MASK, tid);
    }
  }

  // If the atomic TID->0 transition fails (ie FUTEX_WAITERS is set),
  if (!compare_and_swap(&m->futex, tid, 0)) {
    // The kernel handles everything else.
    const int ret = sys_futex_unlock_pi(&m->futex);
    if (ret != 0) {
      PELOG(FATAL, -ret, "FUTEX_UNLOCK_PI(%p) failed", &m->futex);
    }
  } else {
    // There aren't any waiters, so no need to call into the kernel.
  }
}

int mutex_trylock(aos_mutex *m) {
  // Try an atomic 0->TID transition.
  uint32_t c = __sync_val_compare_and_swap(&m->futex, 0, get_tid());

  if (c != 0) {
    // Somebody else had it locked; we failed.
    return 4;
  }
  return 0;
}

bool mutex_islocked(const aos_mutex *m) {
  const uint32_t tid = get_tid();

  const uint32_t value = __atomic_load_n(&m->futex, __ATOMIC_RELAXED);
  return (value & FUTEX_TID_MASK) == tid;
}

int condition_wait(aos_condition *c, aos_mutex *m) {
  const uint32_t wait_start = __atomic_load_n(c, __ATOMIC_SEQ_CST);

  mutex_unlock(m);

  while (true) {
    // Wait in the kernel iff the value of it doesn't change (ie somebody else
    // does a wake) from before we unlocked the mutex.
    int ret;
    ret = sys_futex_wait(FUTEX_WAIT, c, wait_start, nullptr);
    if (ret != 0) {
      // If it failed because somebody else did a wake and changed the value
      // before we actually made it to sleep.
      if (__builtin_expect(ret == -EAGAIN, 1)) {
        // Everything is just normal locks
        // etc, so there's no need to do anything special here.

        // We have to relock it ourself because the kernel didn't do it.
        const int r = mutex_do_get(m, false, nullptr);
        assert(__builtin_expect(r == 0 || r == 1, 1));
        return r;
      }
      // Try again if it was because of a signal.
      if (__builtin_expect(ret == -EINTR, 1)) continue;
      PELOG(FATAL, -ret, "FUTEX_WAIT(%p, %" PRIu32 ", nullptr) failed",
            c, wait_start);
    } else {
      // We have to take the lock ourself because the kernel won't, but
      // there's no need for it to be anything special because all waiters
      // just relock it like usual.
      const int r = mutex_do_get(m, false, nullptr);
      assert(__builtin_expect(r == 0 || r == 1, 1));
      return r;
    }
  }
}

void condition_signal(aos_condition *c, aos_mutex *m) {
  condition_wake(c, m, 0);
}

void condition_broadcast(aos_condition *c, aos_mutex *m) {
  condition_wake(c, m, INT_MAX);
}

int futex_wait_timeout(aos_futex *m, const struct timespec *timeout) {
  if (__atomic_load_n(m, __ATOMIC_SEQ_CST) != 0) {
    return 0;
  }
  const int ret = sys_futex_wait(FUTEX_WAIT, m, 0, timeout);
  if (ret != 0) {
    if (ret == -EINTR) {
      return 1;
    } else if (ret == -ETIMEDOUT) {
      return 2;
    } else if (ret != -EWOULDBLOCK) {
      errno = -ret;
      return -1;
    }
  }
  return 0;
}

int futex_wait(aos_futex *m) { return futex_wait_timeout(m, NULL); }

int futex_set_value(aos_futex *m, uint32_t value) {
  __atomic_store_n(m, value, __ATOMIC_SEQ_CST);
  const int r = sys_futex_wake(m, INT_MAX - 4096);
  if (__builtin_expect(
          static_cast<unsigned int>(r) > static_cast<unsigned int>(-4096), 0)) {
    errno = -r;
    return -1;
  } else {
    return r;
  }
}

int futex_set(aos_futex *m) {
  return futex_set_value(m, 1);
}

int futex_unset(aos_futex *m) {
  return !__atomic_exchange_n(m, 0, __ATOMIC_SEQ_CST);
}
