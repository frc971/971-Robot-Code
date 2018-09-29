#ifndef AOS_LIBC_AOS_STRERROR_H_
#define AOS_LIBC_AOS_STRERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

// Thread-safe version of strerror(3) (except it may change errno).
//
// Returns a pointer to static data or a thread-local buffer.
//
// Necessary because strerror_r(3) is such a mess (which version you get is
// determined at compile time by black magic related to feature macro
// definitions, compiler flags, glibc version, and even whether you're using g++
// or clang++) and strerror_l(3) might not work if you end up with the magic
// LC_GLOBAL_LOCALE locale.
const char *aos_strerror(int error);

#ifdef __cplusplus
}
#endif

#endif  // AOS_LIBC_AOS_STRERROR_H_
