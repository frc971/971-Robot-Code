#ifndef AOS_LIBC_DIRNAME_H_
#define AOS_LIBC_DIRNAME_H_

#include <string>

namespace aos {
namespace libc {

// Thread-safe version of dirname(3).
::std::string Dirname(const ::std::string &path);

}  // namespace libc
}  // namespace aos

#endif  // AOS_LIBC_DIRNAME_H_
