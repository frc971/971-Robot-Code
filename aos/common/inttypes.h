#ifndef AOS_COMMON_INTTYPES_H_
#define AOS_COMMON_INTTYPES_H_

// This file is here because the vxworks headers do not have an inttypes.h file
// and being able to print out fixed size types is very useful. Any fixed size
// formats that we need on the cRIO should get added here.

#ifndef __VXWORKS__
#include <inttypes.h>
#else
// It warns about just "d", but not this, which is kind of weird because
// sizeof(int) == sizeof(long) == sizeof(int32_t) == 4, but oh well.
#define PRId32 "ld"
#define PRIx32 "lx"
#define PRId64 "lld"
#define PRIu16 "u"
#endif

#endif  // AOS_COMMON_INTTYPES_H_
