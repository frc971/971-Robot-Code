#ifndef AOS_COMMON_LIBC_AOS_STRSIGNAL_H_
#define AOS_COMMON_LIBC_AOS_STRSIGNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

// Thread-safe version of strsignal(3) (except it will never return NULL).
//
// Returns a pointer to static data or a thread-local buffer.
const char *aos_strsignal(int signal);

#ifdef __cplusplus
}
#endif

#endif  // AOS_COMMON_LIBC_AOS_STRSIGNAL_H_
