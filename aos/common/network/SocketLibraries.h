// Includes the socket libraries under vxworks and linux.
// Defines a lame_unconst macro for the vxworks functions that need it.

#ifndef __VXWORKS__
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#define lame_unconst(a) a
#else
#include <inetLib.h>
#include <sockLib.h>
#include <selectLib.h>
// Vxworks is missing the const in a couple of its function signatures, so...
#define lame_unconst(a) const_cast<char *>(a)
#endif

