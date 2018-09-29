#ifndef _AOS_MACROS_H_
#define _AOS_MACROS_H_

// This file has some common macros etc.

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete;      \
  void operator=(const TypeName&) = delete

// A macro to wrap arguments to macros that contain commas.
// Useful for DISALLOW_COPY_AND_ASSIGNing templated types with multiple template
// arguments.
#define MACRO_ARG(...) __VA_ARGS__
// Double-wraps macro arguments.
// Necessary to use commas in gtest predicate arguments.
#define MACRO_DARG(...) (__VA_ARGS__)

#ifdef __GNUC__
#define UNUSED_VARIABLE __attribute__ ((unused))
#else
#define UNUSED_VARIABLE
#endif

#define STRINGIFY(x) TO_STRING(x)
#define TO_STRING(x) #x

#ifdef __VXWORKS__
// We're using ancient glibc, so sticking to just what the syscall can handle is
// probably safer.
#define GOOD_PRINTF_FORMAT_TYPE printf
#else
#ifdef __clang__
#define GOOD_PRINTF_FORMAT_TYPE __printf__
#else
#define GOOD_PRINTF_FORMAT_TYPE gnu_printf
#endif
#endif

#endif  // _AOS_MACROS_H_
