#ifndef AOS_LINUX_CODE_THREAD_LOCAL_H_
#define AOS_LINUX_CODE_THREAD_LOCAL_H_

// The storage class to use when declaring thread-local variables. This provides
// a single place to change it if/when we want to switch to something standard.
//
// Example: AOS_THREAD_LOCAL void *bla;  // at namespace (aka global) scope
//
// C++11 has thread_local, but it's not clear whether Clang supports that as of
// 12/18/12.
#define AOS_THREAD_LOCAL __thread

#endif  // AOS_LINUX_CODE_THREAD_LOCAL_H_
