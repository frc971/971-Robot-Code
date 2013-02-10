#include <stdio.h>
#include <linux/futex.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <errno.h>
#include "aos_sync.h"
#include "cmpxchg.h"
#include <stdint.h>
#include <limits.h>
#include <string.h>

// this code is based on something that appears to be based on <http://www.akkadia.org/drepper/futex.pdf>, which also has a lot of useful information
// should probably use <http://lxr.linux.no/linux+v2.6.34/Documentation/robust-futexes.txt> once it becomes available
// <http://locklessinc.com/articles/futex_cheat_sheet/> and <http://locklessinc.com/articles/mutex_cv_futex/> are useful
// <http://lwn.net/Articles/360699/> has a nice overview of futexes in late 2009 (fairly recent compared to everything else...)
// can't use PRIVATE futex operations because they use the pid (or something) as part of the hash
//
// Values for a mutex:
// 0 = unlocked
// 1 = locked, not contended
// 2 = locked, probably contended
// Values for a condition:
// 0 = unset
// 1 = set

static inline int sys_futex(mutex *addr1, int op, int val1, const struct timespec *timeout,
		void *addr2, int val3) {
	return syscall(SYS_futex, addr1, op, val1, timeout, addr2, val3);
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

int mutex_unlock(mutex *m) {
	/* Unlock, and if not contended then exit. */
	//printf("mutex_unlock(%p) => %d \n",m,*m);
	switch (xchg(m, 0)) {
		case 0:
			fprintf(stderr, "sync: multiple unlock of %p. aborting\n", m);
			printf("see stderr\n");
			abort();
		case 1:
			//printf("mutex_unlock return(%p) => %d \n",m,*m);
			return 0;
		case 2:
			if (sys_futex(m, FUTEX_WAKE, 1, NULL, NULL, 0) == -1) {
				fprintf(stderr, "sync: waking 1 from %p failed with %d: %s\n",
						m, errno, strerror(errno));
				return -1;
			} else {
				return 0;
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

int condition_wait(mutex *m) {
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
int condition_wait_force(mutex *m) {
	while (1) {
		if (sys_futex(m, FUTEX_WAIT, *m, NULL, NULL, 0) == -1) {
			if (errno != EWOULDBLOCK) { // if it was an actual problem
				if (errno == EINTR) {
					return 1;
				} else {
					return -1;
				}
			}
		} else {
			return 0;
		}
	}
}
inline int condition_set_value(mutex *m, mutex value) {
	xchg(m, value);
	return sys_futex(m, FUTEX_WAKE, INT_MAX, NULL, NULL, 0);
}
int condition_set(mutex *m) {
	return condition_set_value(m, 1);
}
int condition_unset(mutex *m) {
	return !xchg(m, 0);
}

