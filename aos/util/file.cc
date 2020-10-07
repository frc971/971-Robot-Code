#include "aos/util/file.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <string_view>

#include "aos/scoped/scoped_fd.h"

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

bool MkdirPIfSpace(std::string_view path, mode_t mode) {
  auto last_slash_pos = path.find_last_of("/");

  std::string folder(last_slash_pos == std::string_view::npos
                         ? std::string_view("")
                         : path.substr(0, last_slash_pos));
  if (folder.empty()) {
    return true;
  }
  if (!MkdirPIfSpace(folder, mode)) {
    return false;
  }
  const int result = mkdir(folder.c_str(), mode);
  if (result == -1 && errno == EEXIST) {
    VLOG(2) << folder << " already exists";
    return true;
  } else if (result == -1 && errno == ENOSPC) {
    VLOG(2) << "Out of space";
    return false;
  } else {
    VLOG(1) << "Created " << folder;
  }
  PCHECK(result == 0) << ": Error creating " << folder;
  return true;
}

bool PathExists(std::string_view path) {
  struct stat buffer;
  return stat(path.data(), &buffer) == 0;
}

}  // namespace util
}  // namespace aos
