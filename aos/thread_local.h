#ifndef AOS_THREAD_LOCAL_H_
#define AOS_THREAD_LOCAL_H_

// Use AOS_THREAD_LOCAL instead of thread_local to pick up specifics for various
// compilers/platforms.

#ifdef __aarch64__
// Workaround for https://bugs.llvm.org/show_bug.cgi?id=41527.
// TODO(Brian): Remove this once we upgrade past LLVM 9.0.0.
// 9.0.1 might have the fix, but I can't find prebuilt binaries for it, for some
// reason. Going by release dates, 10.0.0 should definitely have the fix.
//
// https://reviews.llvm.org/D53906 broke it, https://reviews.llvm.org/D62055
// reverted it, https://reviews.llvm.org/D61825 re-enabled it for only Android.
//
// Basically, LLD hacks the program header, but fails to change enough of the
// values to be self-consistent. The resulting values cause glibc's dynamic
// linker to do something different than lld is expecting, so then things
// overlap at runtime and break horribly.
//
// This workaround ensures that the program header has the alignment lld wants
// already, which ensures it's set there early enough in lld's processing that
// it actually gets the proper alignment. This makes the hack a NOP so
// everything works correctly.
//
// To check for the problem, build a binary (a complete binary, not a test
// binary which references shared objects for all the code) and run `readelf
// -aW` on it. Look for the TLS program header. If its alignment is 0x40, this
// workaround is probably needed. Verify its address is aligned mod 0x40 to
// verify the workaround is effective.
#define AOS_THREAD_LOCAL __attribute__((aligned(0x40))) thread_local
#else
#define AOS_THREAD_LOCAL thread_local
#endif

#endif  // AOS_THREAD_LOCAL_H_
