#include "aos/common/util/file.h"

#include <fcntl.h>
#include <unistd.h>

#include "aos/common/scoped_fd.h"

namespace aos {
namespace util {

::std::string ReadFileToStringOrDie(const ::std::string &filename) {
  ::std::string r;
  ScopedFD fd(PCHECK(open(filename.c_str(), O_RDONLY)));
  while (true) {
    char buffer[1024];
    const ssize_t result = read(fd.get(), buffer, sizeof(buffer));
    if (result < 0) {
      PLOG(FATAL, "reading from %s", filename.c_str());
    } else if (result == 0) {
      break;
    }
    r.append(buffer, result);
  }
  return r;
}

}  // namespace util
}  // namespace aos
