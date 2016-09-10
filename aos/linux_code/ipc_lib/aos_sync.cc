#if !AOS_DEBUG
#undef NDEBUG
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

#ifdef AOS_SANITIZER_thread
#include <sanitizer/tsan_interface_atomic.h>
#endif

#include <algorithm>
#include <type_traits>

#include "aos/common/logging/logging.h"
#include "aos/common/once.h"
#include "aos/common/macros.h"
#include "aos/common/util/compiler_memory_barrier.h"

using ::aos::linux_code::ipc_lib::FutexAccessorObserver;

// This code was originally based on <https://www.akkadia.org/drepper/futex.pdf>,
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
// |FUTEX_OWNER_DIED = old owner died
//
// Values for an aos_futex being used directly:
// 0 = unset
// 1 = set
//
// The value of an aos_condition is just a generation counter.

// Whether or not to use the REQUEUE_PI operation. Using it is better (less
// syscalls and the highest priority waiter is always the one that gets woken),
// but there's a kernel bug that results in random memory corruption while using
// them.
// The alternative is to just wake everybody and have them all race to relock
// the mutex (classic thundering herd).
// Currently just whether or not we're not on ARM because we only run this on
// ARM kernels with the patch to fix that issue applied. This will likely change
// to something based on kernel version at some point.
#ifdef __arm__
#define USE_REQUEUE_PI 1
#else
#define USE_REQUEUE_PI 0
#endif

#ifdef AOS_SANITIZER_thread
extern "C" void AnnotateHappensBefore(const char *file, int line,
                                      uintptr_t addr);
extern "C" void AnnotateHappensAfter(const char *file, int line,
                                     uintptr_t addr);
#define ANNOTATE_HAPPENS_BEFORE(address)    \
  AnnotateHappensBefore(__FILE__, __LINE__, \
                        reinterpret_cast<uintptr_t>(address))
#define ANNOTATE_HAPPENS_AFTER(address) \
  AnnotateHappensAfter(__FILE__, __LINE__, reinterpret_cast<uintptr_t>(address))
#else
#define ANNOTATE_HAPPENS_BEFORE(address)
#define ANNOTATE_HAPPENS_AFTER(address)
#endif

namespace {

const bool kRobustListDebug = false;
const bool kLockDebug = false;
const bool kPrintOperations = false;

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

inline int sys_futex_cmp_requeue_pi(aos_futex *addr1, int num_wake,
    int num_requeue, aos_futex *m, uint32_t val) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_CMP_REQUEUE_PI;
  register int num_wake_reg __asm__("r2") = num_wake;
  register int num_requeue_reg __asm__("r3") = num_requeue;
  register aos_futex *m_reg __asm__("r4") = m;
  register uint32_t val_reg __asm__("r5") = val;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(num_wake_reg),
                     "r"(num_requeue_reg), "r"(m_reg), "r"(val_reg),
                     "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, FUTEX_CMP_REQUEUE_PI, num_wake,
                        num_requeue, m, val);
  if (r == -1) return -errno;
  return r;
#endif
}

inline int sys_futex_wait_requeue_pi(aos_condition *addr1,
                                     uint32_t start_val,
                                     const struct timespec *timeout,
                                     aos_futex *m) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_condition *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_WAIT_REQUEUE_PI;
  register uint32_t start_val_reg __asm__("r2") = start_val;
  register const struct timespec *timeout_reg __asm__("r3") = timeout;
  register aos_futex *m_reg __asm__("r4") = m;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(start_val_reg),
                     "r"(timeout_reg), "r"(m_reg), "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r =
      syscall(SYS_futex, addr1, FUTEX_WAIT_REQUEUE_PI, start_val, timeout, m);
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

// Returns the previous value of f.
inline uint32_t compare_and_swap_val(aos_futex *f, uint32_t before,
                                     uint32_t after) {
#ifdef AOS_SANITIZER_thread
  // This is a workaround for <https://llvm.org/bugs/show_bug.cgi?id=23176>.
  // Basically, most of the atomic operations are broken under tsan, but this
  // particular one isn't.
  // TODO(Brian): Remove this #ifdef (and the one in compare_and_swap) once we
  // don't have to worry about tsan with this bug any more.
  uint32_t before_value = before;
  __tsan_atomic32_compare_exchange_strong(
      reinterpret_cast<int32_t *>(f),
      reinterpret_cast<int32_t *>(&before_value), after,
      __tsan_memory_order_seq_cst, __tsan_memory_order_seq_cst);
  return before_value;
#else
  return __sync_val_compare_and_swap(f, before, after);
#endif
}

// Returns true if it succeeds and false if it fails.
inline bool compare_and_swap(aos_futex *f, uint32_t before, uint32_t after) {
#ifdef AOS_SANITIZER_thread
  return compare_and_swap_val(f, before, after) == before;
#else
  return __sync_bool_compare_and_swap(f, before, after);
#endif
}

#ifdef AOS_SANITIZER_thread

// Simple macro for checking something which should always be true.
// Using the standard CHECK macro isn't safe because failures often result in
// reentering the mutex locking code, which doesn't work.
#define SIMPLE_CHECK(expr)                                                   \
  do {                                                                       \
    if (!(expr)) {                                                           \
      fprintf(stderr, "%s: %d: SIMPLE_CHECK(" #expr ") failed!\n", __FILE__, \
              __LINE__);                                                     \
      abort();                                                               \
    }                                                                        \
  } while (false)

// Forcibly initializes the pthread mutex for *m.
// This sequence of operations is only safe for the simpler kinds of mutexes in
// glibc's pthreads implementation on Linux.
void init_pthread_mutex(aos_mutex *m) {
  // Re-initialize the mutex so the destroy won't fail if it's locked.
  // tsan ignores this.
  SIMPLE_CHECK(0 == pthread_mutex_init(&m->pthread_mutex, nullptr));
  // Destroy the mutex so tsan will forget about it if some now-dead thread
  // locked it.
  SIMPLE_CHECK(0 == pthread_mutex_destroy(&m->pthread_mutex));

  // Now actually initialize it, making sure it's process-shareable so it works
  // correctly across shared memory.
  pthread_mutexattr_t attr;
  SIMPLE_CHECK(0 == pthread_mutexattr_init(&attr));
  SIMPLE_CHECK(0 == pthread_mutexattr_setpshared(&attr, true));
  SIMPLE_CHECK(0 == pthread_mutex_init(&m->pthread_mutex, &attr));
  SIMPLE_CHECK(0 == pthread_mutexattr_destroy(&attr));
}

// Locks the pthread mutex for *m.
// If a stack trace ever reveals the pthread_mutex_lock call in here blocking,
// there is a bug in our mutex code or the way somebody is calling it.
void lock_pthread_mutex(aos_mutex *m) {
  if (!m->pthread_mutex_init) {
    init_pthread_mutex(m);
    m->pthread_mutex_init = true;
  }
  SIMPLE_CHECK(0 == pthread_mutex_lock(&m->pthread_mutex));
}

// Forcibly locks the pthread mutex for *m.
// This will (somewhat hackily) rip the lock out from underneath somebody else
// who is already holding it.
void force_lock_pthread_mutex(aos_mutex *m) {
  if (!m->pthread_mutex_init) {
    init_pthread_mutex(m);
    m->pthread_mutex_init = true;
  }
  const int trylock_result = pthread_mutex_trylock(&m->pthread_mutex);
  SIMPLE_CHECK(trylock_result == 0 || trylock_result == EBUSY);
  if (trylock_result == 0) {
    // We're good, so unlock it and then go for a real lock down below.
    SIMPLE_CHECK(0 == pthread_mutex_unlock(&m->pthread_mutex));
  } else {
    // Somebody (should always be somebody else who died with it held) already
    // has it, so make tsan forget about that.
    init_pthread_mutex(m);
  }
  lock_pthread_mutex(m);
}

// Unlocks the pthread mutex for *m.
void unlock_pthread_mutex(aos_mutex *m) {
  assert(m->pthread_mutex_init);
  SIMPLE_CHECK(0 == pthread_mutex_unlock(&m->pthread_mutex));
}

#else

// Empty implementations of all these so the code below doesn't need #ifdefs.
static inline void lock_pthread_mutex(aos_mutex *) {}
static inline void force_lock_pthread_mutex(aos_mutex *) {}
static inline void unlock_pthread_mutex(aos_mutex *) {}

#endif

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
thread_local pid_t my_tid = 0;

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
  if (__builtin_expect(my_tid == 0, false)) {
    initialize_in_new_thread();
  }
  static_assert(sizeof(my_tid) <= sizeof(uint32_t), "pid_t is too big");
  return static_cast<uint32_t>(my_tid);
}

// Contains all of the stuff for dealing with the robust list. Nothing outside
// this namespace should touch anything inside it except Init, Adder, and
// Remover.
namespace my_robust_list {

static_assert(offsetof(aos_mutex, next) == 0,
              "Our math all assumes that the beginning of a mutex and its next "
              "pointer are at the same place in memory.");

// Our version of robust_list_head.
// This is copied from the kernel header because that's a pretty stable ABI (and
// any changes will be backwards compatible anyways) and we want ours to have
// different types.
// The uintptr_ts are &next of the elements in the list (with stuff |ed in).
struct aos_robust_list_head {
  uintptr_t next;
  long futex_offset;
  uintptr_t pending_next;
};

static_assert(offsetof(aos_robust_list_head, next) ==
                  offsetof(robust_list_head, list),
              "Our aos_robust_list_head doesn't match the kernel's");
static_assert(offsetof(aos_robust_list_head, futex_offset) ==
                  offsetof(robust_list_head, futex_offset),
              "Our aos_robust_list_head doesn't match the kernel's");
static_assert(offsetof(aos_robust_list_head, pending_next) ==
                  offsetof(robust_list_head, list_op_pending),
              "Our aos_robust_list_head doesn't match the kernel's");
static_assert(sizeof(aos_robust_list_head) == sizeof(robust_list_head),
              "Our aos_robust_list_head doesn't match the kernel's");

thread_local aos_robust_list_head robust_head;

// Extra offset between mutex values and where we point to for their robust list
// entries (from SetRobustListOffset).
uintptr_t robust_list_offset = 0;

// The value to OR each pointer's value with whenever putting it into the robust
// list (technically only if it's PI, but all of ours are, so...).
static const uintptr_t kRobustListOr = 1;

// Returns the value which goes into a next variable to represent the head.
inline uintptr_t robust_head_next_value() {
  return reinterpret_cast<uintptr_t>(&robust_head.next);
}
// Returns true iff next represents the head.
inline bool next_is_head(uintptr_t next) {
  return next == robust_head_next_value();
}
// Returns the (psuedo-)mutex corresponding to the head.
// This does NOT have a previous pointer, so be careful with the return value.
inline aos_mutex *robust_head_mutex() {
  return reinterpret_cast<aos_mutex *>(robust_head_next_value());
}

inline uintptr_t mutex_to_next(aos_mutex *m) {
  return (reinterpret_cast<uintptr_t>(&m->next) + robust_list_offset) |
         kRobustListOr;
}
inline aos_mutex *next_to_mutex(uintptr_t next) {
  if (__builtin_expect(robust_list_offset != 0, false) && next_is_head(next)) {
    // We don't offset the head pointer, so be careful.
    return reinterpret_cast<aos_mutex *>(next);
  }
  return reinterpret_cast<aos_mutex *>(
      (next & ~kRobustListOr) - robust_list_offset);
}

// Sets up the robust list for each thread.
void Init() {
  // It starts out just pointing back to itself.
  robust_head.next = robust_head_next_value();
  robust_head.futex_offset = static_cast<ssize_t>(offsetof(aos_mutex, futex)) -
                             static_cast<ssize_t>(offsetof(aos_mutex, next));
  robust_head.pending_next = 0;
  if (syscall(SYS_set_robust_list, robust_head_next_value(), sizeof(robust_head)) !=
      0) {
    PLOG(FATAL, "set_robust_list(%p, %zd) failed",
         reinterpret_cast<void *>(robust_head.next), sizeof(robust_head));
  }
  if (kRobustListDebug) {
    printf("%" PRId32 ": init done\n", get_tid());
  }
}

// Updating the offset with locked mutexes is important during robustness
// testing, because there are mutexes which are locked before this is set to a
// non-0 value and then unlocked after it is changed back. However, to make sure
// the code works correctly when manipulating the next pointer of the last of
// those mutexes, all of their next values have to be adjusted appropriately.
void SetRobustListOffset(uintptr_t offset) {
  const uintptr_t offset_change = offset - robust_list_offset;
  robust_list_offset = offset;
  aos_mutex *m = robust_head_mutex();
  // Update the offset contained in each of the mutexes which is already locked.
  while (!next_is_head(m->next)) {
    m->next += offset_change;
    m = next_to_mutex(m->next);
  }
}

bool HaveLockedMutexes() {
  return robust_head.next != robust_head_next_value();
}

// Handles adding a mutex to the robust list.
// The idea is to create one of these at the beginning of a function that needs
// to do this and then call Add() iff it should actually be added.
class Adder {
 public:
  Adder(aos_mutex *m) : m_(m) {
    assert(robust_head.pending_next == 0);
    if (kRobustListDebug) {
      printf("%" PRId32 ": maybe add %p\n", get_tid(), m_);
    }
    robust_head.pending_next = mutex_to_next(m);
    aos_compiler_memory_barrier();
  }
  ~Adder() {
    assert(robust_head.pending_next == mutex_to_next(m_));
    if (kRobustListDebug) {
      printf("%" PRId32 ": done maybe add %p, n=%p p=%p\n", get_tid(), m_,
             next_to_mutex(m_->next), m_->previous);
    }
    aos_compiler_memory_barrier();
    robust_head.pending_next = 0;
  }

  void Add() {
    assert(robust_head.pending_next == mutex_to_next(m_));
    if (kRobustListDebug) {
      printf("%" PRId32 ": adding %p\n", get_tid(), m_);
    }
    const uintptr_t old_head_next_value = robust_head.next;

    m_->next = old_head_next_value;
    aos_compiler_memory_barrier();
    robust_head.next = mutex_to_next(m_);

    m_->previous = robust_head_mutex();
    if (!next_is_head(old_head_next_value)) {
      // robust_head's psuedo-mutex doesn't have a previous pointer to update.
      next_to_mutex(old_head_next_value)->previous = m_;
    }
    aos_compiler_memory_barrier();
    if (kRobustListDebug) {
      printf("%" PRId32 ": done adding %p\n", get_tid(), m_);
    }
  }

 private:
  aos_mutex *const m_;

  DISALLOW_COPY_AND_ASSIGN(Adder);
};

// Handles removing a mutex from the robust list.
// The idea is to create one of these at the beginning of a function that needs
// to do this.
class Remover {
 public:
  Remover(aos_mutex *m) {
    assert(robust_head.pending_next == 0);
    if (kRobustListDebug) {
      printf("%" PRId32 ": beginning to remove %p, n=%p p=%p\n", get_tid(), m,
             next_to_mutex(m->next), m->previous);
    }
    robust_head.pending_next = mutex_to_next(m);
    aos_compiler_memory_barrier();

    aos_mutex *const previous = m->previous;
    const uintptr_t next_value = m->next;

    previous->next = m->next;
    if (!next_is_head(next_value)) {
      // robust_head's psuedo-mutex doesn't have a previous pointer to update.
      next_to_mutex(next_value)->previous = previous;
    }

    if (kRobustListDebug) {
      printf("%" PRId32 ": done removing %p\n", get_tid(), m);
    }
  }
  ~Remover() {
    assert(robust_head.pending_next != 0);
    aos_compiler_memory_barrier();
    robust_head.pending_next = 0;
    if (kRobustListDebug) {
      printf("%" PRId32 ": done with removal\n", get_tid());
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(Remover);
};

}  // namespace my_robust_list

void initialize_in_new_thread() {
  // No synchronization necessary in most of this because it's all thread-local!

  my_tid = do_get_tid();

  static ::aos::Once<void> atfork_hook_installed(InstallAtforkHook);
  atfork_hook_installed.Get();

  my_robust_list::Init();
}

FutexAccessorObserver before_observer = nullptr, after_observer = nullptr;

// RAII class which runs before_observer during construction and after_observer
// during destruction.
class RunObservers {
 public:
  template <class T>
  RunObservers(T *address, bool write)
      : address_(static_cast<void *>(
            const_cast<typename ::std::remove_cv<T>::type *>(address))),
        write_(write) {
    if (__builtin_expect(before_observer != nullptr, false)) {
      before_observer(address_, write_);
    }
  }
  ~RunObservers() {
    if (__builtin_expect(after_observer != nullptr, false)) {
      after_observer(address_, write_);
    }
  }

 private:
  void *const address_;
  const bool write_;

  DISALLOW_COPY_AND_ASSIGN(RunObservers);
};

// Finishes the locking of a mutex by potentially clearing FUTEX_OWNER_DIED in
// the futex and returning the correct value.
inline int mutex_finish_lock(aos_mutex *m) {
  const uint32_t value = __atomic_load_n(&m->futex, __ATOMIC_ACQUIRE);
  if (__builtin_expect((value & FUTEX_OWNER_DIED) != 0, false)) {
    __atomic_and_fetch(&m->futex, ~FUTEX_OWNER_DIED, __ATOMIC_RELAXED);
    force_lock_pthread_mutex(m);
    return 1;
  } else {
    lock_pthread_mutex(m);
    return 0;
  }
}

// Split out separately from mutex_get so condition_wait can call it and use its
// own my_robust_list::Adder.
inline int mutex_do_get(aos_mutex *m, bool signals_fail,
                        const struct timespec *timeout, uint32_t tid) {
  RunObservers run_observers(m, true);
  if (kPrintOperations) {
    printf("%" PRId32 ": %p do_get\n", tid, m);
  }

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
        if (__builtin_expect(ret == -EINTR, true)) {
          if (signals_fail) {
            return 2;
          } else {
            continue;
          }
        }
        my_robust_list::robust_head.pending_next = 0;
        if (ret == -EDEADLK) {
          LOG(FATAL, "multiple lock of %p by %" PRId32 "\n", m, tid);
        }
        PELOG(FATAL, -ret, "FUTEX_LOCK_PI(%p(=%" PRIu32 "), 1, %p) failed",
              &m->futex, __atomic_load_n(&m->futex, __ATOMIC_SEQ_CST), timeout);
      } else {
        if (kLockDebug) {
          printf("%" PRId32 ": %p kernel lock done\n", tid, m);
        }
        // The kernel already handled setting the value to our TID (ish).
        break;
      }
    } else {
      if (kLockDebug) {
        printf("%" PRId32 ": %p fast lock done\n", tid, m);
      }
      lock_pthread_mutex(m);
      // Fastpath succeeded, so no need to call into the kernel.
      // Because this is the fastpath, it's a good idea to avoid even having to
      // load the value again down below.
      return 0;
    }
  }

  return mutex_finish_lock(m);
}

// The common implementation for everything that wants to lock a mutex.
// If signals_fail is false, the function will try again if the wait syscall is
// interrupted by a signal.
// timeout can be NULL for no timeout.
inline int mutex_get(aos_mutex *m, bool signals_fail,
                     const struct timespec *timeout) {
  const uint32_t tid = get_tid();
  my_robust_list::Adder adder(m);
  const int r = mutex_do_get(m, signals_fail, timeout, tid);
  if (r == 0 || r == 1) adder.Add();
  return r;
}

// The common implementation for broadcast and signal.
// number_requeue is the number of waiters to requeue (probably INT_MAX or 0). 1
// will always be woken.
void condition_wake(aos_condition *c, aos_mutex *m, int number_requeue) {
  RunObservers run_observers(c, true);
  // Make it so that anybody just going to sleep won't.
  // This is where we might accidentally wake more than just 1 waiter with 1
  // signal():
  //   1 already sleeping will be woken but n might never actually make it to
  //     sleep in the kernel because of this.
  uint32_t new_value = __atomic_add_fetch(c, 1, __ATOMIC_SEQ_CST);

  if (USE_REQUEUE_PI) {
    while (true) {
      // This really wants to be FUTEX_REQUEUE_PI, but the kernel doesn't have
      // that... However, the code to support that is in the kernel, so it might
      // be a good idea to patch it to support that and use it iff it's there.
      const int ret =
          sys_futex_cmp_requeue_pi(c, 1, number_requeue, &m->futex, new_value);
      if (ret < 0) {
        // If the value got changed out from under us (aka somebody else did a
        // condition_wake).
        if (__builtin_expect(ret == -EAGAIN, true)) {
          // If we're doing a broadcast, the other guy might have done a signal
          // instead, so we have to try again.
          // If we're doing a signal, we have to go again to make sure that 2
          // signals wake 2 processes.
          new_value = __atomic_load_n(c, __ATOMIC_RELAXED);
          continue;
        }
        my_robust_list::robust_head.pending_next = 0;
        PELOG(FATAL, -ret, "FUTEX_CMP_REQUEUE_PI(%p, 1, %d, %p, *%p) failed",
              c, number_requeue, &m->futex, c);
      } else {
        return;
      }
    }
  } else {
    const int ret = sys_futex_wake(
        c, ::std::min(::std::max(number_requeue, 1), INT_MAX - 4096));
    if (__builtin_expect(
            static_cast<unsigned int>(ret) > static_cast<unsigned int>(-4096),
            false)) {
      my_robust_list::robust_head.pending_next = 0;
      PELOG(FATAL, -ret, "FUTEX_WAKE(%p, %d) failed", c, INT_MAX - 4096);
    }
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
  RunObservers run_observers(m, true);
  const uint32_t tid = get_tid();
  if (kPrintOperations) {
    printf("%" PRId32 ": %p unlock\n", tid, m);
  }

  const uint32_t value = __atomic_load_n(&m->futex, __ATOMIC_SEQ_CST);
  if (__builtin_expect((value & FUTEX_TID_MASK) != tid, false)) {
    my_robust_list::robust_head.pending_next = 0;
    check_cached_tid(tid);
    if ((value & FUTEX_TID_MASK) == 0) {
      LOG(FATAL, "multiple unlock of aos_mutex %p by %" PRId32 "\n", m, tid);
    } else {
      LOG(FATAL, "aos_mutex %p is locked by %" PRId32 ", not %" PRId32 "\n",
          m, value & FUTEX_TID_MASK, tid);
    }
  }

  my_robust_list::Remover remover(m);
  unlock_pthread_mutex(m);

  // If the atomic TID->0 transition fails (ie FUTEX_WAITERS is set),
  if (!compare_and_swap(&m->futex, tid, 0)) {
    // The kernel handles everything else.
    const int ret = sys_futex_unlock_pi(&m->futex);
    if (ret != 0) {
      my_robust_list::robust_head.pending_next = 0;
      PELOG(FATAL, -ret, "FUTEX_UNLOCK_PI(%p) failed", &m->futex);
    }
  } else {
    // There aren't any waiters, so no need to call into the kernel.
  }
}

int mutex_trylock(aos_mutex *m) {
  RunObservers run_observers(m, true);
  const uint32_t tid = get_tid();
  if (kPrintOperations) {
    printf("%" PRId32 ": %p trylock\n", tid, m);
  }
  my_robust_list::Adder adder(m);

  // Try an atomic 0->TID transition.
  uint32_t c = compare_and_swap_val(&m->futex, 0, tid);

  if (c != 0) {
    if (__builtin_expect((c & FUTEX_OWNER_DIED) == 0, true)) {
      // Somebody else had it locked; we failed.
      return 4;
    } else {
      // FUTEX_OWNER_DIED was set, so we have to call into the kernel to deal
      // with resetting it.
      const int ret = sys_futex_wait(FUTEX_TRYLOCK_PI, &m->futex, 0, NULL);
      if (ret == 0) {
        adder.Add();
        // Only clear the owner died if somebody else didn't do the recovery
        // and then unlock before our TRYLOCK happened.
        return mutex_finish_lock(m);
      } else {
        // EWOULDBLOCK means that somebody else beat us to it.
        if (__builtin_expect(ret == -EWOULDBLOCK, true)) {
          return 4;
        }
        my_robust_list::robust_head.pending_next = 0;
        PELOG(FATAL, -ret, "FUTEX_TRYLOCK_PI(%p, 0, NULL) failed", &m->futex);
      }
    }
  }

  lock_pthread_mutex(m);
  adder.Add();
  return 0;
}

bool mutex_islocked(const aos_mutex *m) {
  const uint32_t tid = get_tid();

  const uint32_t value = __atomic_load_n(&m->futex, __ATOMIC_RELAXED);
  return (value & FUTEX_TID_MASK) == tid;
}

int condition_wait(aos_condition *c, aos_mutex *m) {
  RunObservers run_observers(c, false);
  const uint32_t tid = get_tid();
  const uint32_t wait_start = __atomic_load_n(c, __ATOMIC_SEQ_CST);

  mutex_unlock(m);

  my_robust_list::Adder adder(m);

  while (true) {
    // Wait in the kernel iff the value of it doesn't change (ie somebody else
    // does a wake) from before we unlocked the mutex.
    int ret;
    if (USE_REQUEUE_PI) {
      ret = sys_futex_wait_requeue_pi(c, wait_start, nullptr, &m->futex);
    } else {
      ret = sys_futex_wait(FUTEX_WAIT, c, wait_start, nullptr);
    }
    if (ret != 0) {
      // If it failed because somebody else did a wake and changed the value
      // before we actually made it to sleep.
      if (__builtin_expect(ret == -EAGAIN, true)) {
        // There's no need to unconditionally set FUTEX_WAITERS here if we're
        // using REQUEUE_PI because the kernel automatically does that in the
        // REQUEUE_PI iff it requeued anybody.
        // If we're not using REQUEUE_PI, then everything is just normal locks
        // etc, so there's no need to do anything special there either.

        // We have to relock it ourself because the kernel didn't do it.
        const int r = mutex_do_get(m, false, nullptr, tid);
        assert(__builtin_expect(r == 0 || r == 1, true));
        adder.Add();
        return r;
      }
      // Try again if it was because of a signal.
      if (__builtin_expect(ret == -EINTR, true)) continue;
      my_robust_list::robust_head.pending_next = 0;
      if (USE_REQUEUE_PI) {
        PELOG(FATAL, -ret, "FUTEX_WAIT_REQUEUE_PI(%p, %" PRIu32 ", %p) failed",
              c, wait_start, &m->futex);
      } else {
        PELOG(FATAL, -ret, "FUTEX_WAIT(%p, %" PRIu32 ", nullptr) failed",
              c, wait_start);
      }
    } else {
      if (USE_REQUEUE_PI) {
        // Record that the kernel relocked it for us.
        lock_pthread_mutex(m);
      } else {
        // We have to take the lock ourself because the kernel won't, but
        // there's no need for it to be anything special because all waiters
        // just relock it like usual.
        const int r = mutex_do_get(m, false, nullptr, tid);
        assert(__builtin_expect(r == 0 || r == 1, true));
        adder.Add();
        return r;
      }

      // We succeeded in waiting, and the kernel took care of locking the mutex
      // for us and setting FUTEX_WAITERS iff it needed to (for REQUEUE_PI).

      adder.Add();

      const uint32_t value = __atomic_load_n(&m->futex, __ATOMIC_RELAXED);
      if (__builtin_expect((value & FUTEX_OWNER_DIED) != 0, false)) {
        __atomic_and_fetch(&m->futex, ~FUTEX_OWNER_DIED, __ATOMIC_RELAXED);
        return 1;
      } else {
        return 0;
      }
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
  RunObservers run_observers(m, false);
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
  ANNOTATE_HAPPENS_AFTER(m);
  return 0;
}

int futex_wait(aos_futex *m) { return futex_wait_timeout(m, NULL); }

int futex_set_value(aos_futex *m, uint32_t value) {
  RunObservers run_observers(m, false);
  ANNOTATE_HAPPENS_BEFORE(m);
  __atomic_store_n(m, value, __ATOMIC_SEQ_CST);
  const int r = sys_futex_wake(m, INT_MAX - 4096);
  if (__builtin_expect(
          static_cast<unsigned int>(r) > static_cast<unsigned int>(-4096),
          false)) {
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

namespace aos {
namespace linux_code {
namespace ipc_lib {

// Sets functions to run befor eand after all futex operations.
// This is important when doing robustness testing because the memory has to be
// made writable for the whole futex operation, otherwise it never succeeds.
void SetFutexAccessorObservers(FutexAccessorObserver before,
                               FutexAccessorObserver after) {
  before_observer = before;
  after_observer = after;
}

// Sets an extra offset between mutexes and the value we use for them in the
// robust list (only the forward pointers). This is used to work around a kernel
// bug by keeping a second set of mutexes which is always writable so the kernel
// won't go into an infinite loop when trying to unlock them.
void SetRobustListOffset(ptrdiff_t offset) {
  my_robust_list::SetRobustListOffset(offset);
}

// Returns true iff there are any mutexes locked by the current thread.
// This is mainly useful for testing.
bool HaveLockedMutexes() {
  return my_robust_list::HaveLockedMutexes();
}

}  // namespace ipc_lib
}  // namespace linux_code
}  // namespace aos
