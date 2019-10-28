#include "aos/util/file.h"

#include <fcntl.h>
#include <unistd.h>

#include "absl/strings/string_view.h"
#include "aos/logging/logging.h"
#include "aos/scoped/scoped_fd.h"

namespace aos {
namespace util {

::std::string ReadFileToStringOrDie(const absl::string_view filename) {
  ::std::string r;
  ScopedFD fd(open(::std::string(filename).c_str(), O_RDONLY));
  if (fd.get() == -1) {
    AOS_PLOG(FATAL, "opening %*s", static_cast<int>(filename.size()),
             filename.data());
  }
  while (true) {
    char buffer[1024];
    const ssize_t result = read(fd.get(), buffer, sizeof(buffer));
    if (result < 0) {
      AOS_PLOG(FATAL, "reading from %*s", static_cast<int>(filename.size()),
               filename.data());
    } else if (result == 0) {
      break;
    }
    r.append(buffer, result);
  }
  return r;
}

}  // namespace util
}  // namespace aos
