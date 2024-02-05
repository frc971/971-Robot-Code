#ifndef AOS_LIBC_DIRNAME_H_
#define AOS_LIBC_DIRNAME_H_

#include <string>

namespace aos::libc {

// Thread-safe version of dirname(3).
::std::string Dirname(const ::std::string &path);

}  // namespace aos::libc

#endif  // AOS_LIBC_DIRNAME_H_
