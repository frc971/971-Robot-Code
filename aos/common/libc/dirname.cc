#include "aos/common/libc/dirname.h"

namespace aos {
namespace libc {
namespace {

::std::string DoDirname(const ::std::string &path, size_t last_slash) {
  // If there aren't any other '/'s in it.
  if (last_slash == ::std::string::npos) return ".";

  // Back up as long as we see '/'s.
  do {
    // If we get all the way to the beginning.
    if (last_slash == 0) return "/";
    --last_slash;
  } while (path[last_slash] == '/');

  return path.substr(0, last_slash + 1);
}

}  // namespace

::std::string Dirname(const ::std::string &path) {
  // Without this, we end up with integer underflows below, which is technically
  // undefined.
  if (path.size() == 0) return ".";

  size_t last_slash = path.rfind('/');

  // If the path ends with a '/'.
  if (last_slash == path.size() - 1) {
    last_slash = DoDirname(path, last_slash).rfind('/');
  }

  return DoDirname(path, last_slash);
}

}  // namespace libc
}  // namespace aos
