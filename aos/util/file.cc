#include "aos/util/file.h"

#include <fcntl.h>
#include <unistd.h>

#include <string_view>

#include "aos/scoped/scoped_fd.h"
#include "glog/logging.h"

namespace aos {
namespace util {

::std::string ReadFileToStringOrDie(const std::string_view filename) {
  ::std::string r;
  ScopedFD fd(open(::std::string(filename).c_str(), O_RDONLY));
  PCHECK(fd.get() != -1) << ": opening " << filename;
  while (true) {
    char buffer[1024];
    const ssize_t result = read(fd.get(), buffer, sizeof(buffer));
    PCHECK(result >= 0) << ": reading from " << filename;
    if (result == 0) {
      break;
    }
    r.append(buffer, result);
  }
  return r;
}

void WriteStringToFileOrDie(const std::string_view filename,
                            const std::string_view contents) {
  ::std::string r;
  ScopedFD fd(open(::std::string(filename).c_str(),
                   O_CREAT | O_WRONLY | O_TRUNC, S_IRWXU));
  PCHECK(fd.get() != -1) << ": opening " << filename;
  size_t size_written = 0;
  while (size_written != contents.size()) {
    const ssize_t result = write(fd.get(), contents.data() + size_written,
                                 contents.size() - size_written);
    PCHECK(result >= 0) << ": reading from " << filename;
    if (result == 0) {
      break;
    }

    size_written += result;
  }
}

}  // namespace util
}  // namespace aos
