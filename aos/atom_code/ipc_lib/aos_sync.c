#include "aos/atom_code/ipc_lib/aos_sync.h"

#include <stdio.h>
#include <linux/futex.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <errno.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <inttypes.h>

#include "cmpxchg.h"

// this code is based on something that appears to be based on <http://www.akkadia.org/drepper/futex.pdf>, which also has a lot of useful information
// should probably use <http://lxr.linux.no/linux+v2.6.34/Documentation/robust-futexes.txt> once it becomes available
//   (sys_set_robust_list appears to be the function name)
// <http://locklessinc.com/articles/futex_cheat_sheet/> and
//   <http://locklessinc.com/articles/mutex_cv_futex/> are useful
// <http://lwn.net/Articles/360699/> has a nice overview of futexes in late 2009 (fairly recent compared to everything else...)
// can't use PRIVATE futex operations because they use the pid (or something) as part of the hash
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

static inline int sys_futex(mutex *addr1, int op, int val1,
    const struct timespec *timeout, void *addr2, int val3) {
  return syscall(SYS_futex, addr1, op, val1, timeout, addr2, val3);
}
static inline int sys_futex_requeue(mutex *addr1, int op, int num_wake,
    int num_requeue, mutex *m) {
  return syscall(SYS_futex, addr1, op, num_wake, num_requeue, m);
}
static inline int sys_futex_op(mutex *addr1, int op, int num_waiters1,
    int num_waiters2, mutex *addr2, int op_args_etc) {
  return syscall(SYS_futex, addr1, op, num_waiters1,
      num_waiters2, addr2, op_args_etc);
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
    //printf("sync here %d\n", __LINE__);
    if (sys_futex(m, FUTEX_WAIT, 2, timeout, NULL, 0) == -1) {
      if (signals_fail && errno == EINTR) {
        return 1;
      }
      if (timeout != NULL && errno == ETIMEDOUT) {
        return 2;
      }
    }
    //printf("sync here %d\n", __LINE__);
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
  //printf("mutex_unlock(%p) => %d \n",m,*m);
  switch (xchg(m, 0)) {
    case 0:
      fprintf(stderr, "sync: multiple unlock of %p. aborting\n", m);
      printf("see stderr\n");
      abort();
    case 1:
      //printf("mutex_unlock return(%p) => %d \n",m,*m);
      break;
    case 2:
      if (sys_futex(m, FUTEX_WAKE, 1, NULL, NULL, 0) == -1) {
        fprintf(stderr, "sync: waking 1 from %p failed with %d: %s\n",
            m, errno, strerror(errno));
        printf("see stderr\n");
        abort();
      } else {
        break;
      }
    default:
      fprintf(stderr, "sync: got a garbage value from mutex %p. aborting\n",
          m);
      printf("see stderr\n");
      abort();
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
  if (sys_futex(m, FUTEX_WAIT, 0, NULL, NULL, 0) == -1) {
    if (errno == EINTR) {
      return 1;
    } else if (errno != EWOULDBLOCK) {
      return -1;
    }
  }
  return 0;
}
int futex_set_value(mutex *m, mutex value) {
  xchg(m, value);
  return sys_futex(m, FUTEX_WAKE, INT_MAX, NULL, NULL, 0);
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
    if (sys_futex(c, FUTEX_WAIT, wait_start, NULL, NULL, 0) == -1) {
      // If it failed for some reason other than somebody else doing a wake
      // before we actually made it to sleep.
      if (__builtin_expect(*c == wait_start, 0)) {
        // Try again if it was because of a signal.
        if (errno == EINTR) continue;
        fprintf(stderr, "FUTEX_WAIT(%p, %"PRIu32", NULL, NULL, 0) failed"
                " with %d: %s\n",
                c, wait_start, errno, strerror(errno));
        printf("see stderr\n");
        abort();
      }
    }
    // Simplified mutex_lock that always leaves it
    // contended in case anybody else got requeued.
    while (xchg(m, 2) != 0) {
      if (sys_futex(m, FUTEX_WAIT, 2, NULL, NULL, 0) == -1) {
        // Try again if it was because of a signal or somebody else unlocked it
        // before we went to sleep.
        if (errno == EINTR || errno == EWOULDBLOCK) continue;
        fprintf(stderr, "sync: FUTEX_WAIT(%p, 2, NULL, NULL, 0)"
                " failed with %d: %s\n",
                m, errno, strerror(errno));
        printf("see stderr\n");
        abort();
      }
    }
    return;
  }
}

void condition_signal(mutex *c) {
  __sync_fetch_and_add(c, 1);
  if (sys_futex(c, FUTEX_WAKE, 1, NULL, NULL, 0) == -1) {
    fprintf(stderr, "sync: FUTEX_WAKE(%p, 1, NULL, NULL, 0)"
        " failed with %d: %s\n",
        c, errno, strerror(errno));
    printf("see stderr\n");
    abort();
  }
}

void condition_broadcast(mutex *c, mutex *m) {
  __sync_fetch_and_add(c, 1);
  // Wake 1 waiter and requeue the rest.
  if (sys_futex_requeue(c, FUTEX_REQUEUE, 1, INT_MAX, m) == -1) {
    fprintf(stderr, "sync: FUTEX_REQUEUE(%p, 1, INT_MAX, %p, 0)"
        " failed with %d: %s\n",
        c, m, errno, strerror(errno));
    printf("see stderr\n");
    abort();
  }
}
