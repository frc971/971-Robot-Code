#include "aos/util/file.h"

#include <fcntl.h>
#include <fts.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <string_view>
#if __has_feature(memory_sanitizer)
#include <sanitizer/msan_interface.h>
#endif

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
                            const std::string_view contents,
                            mode_t permissions) {
  ::std::string r;
  ScopedFD fd(open(::std::string(filename).c_str(),
                   O_CREAT | O_WRONLY | O_TRUNC, permissions));
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

void UnlinkRecursive(std::string_view path) {
  FTS *ftsp = NULL;
  FTSENT *curr;

  // Cast needed (in C) because fts_open() takes a "char * const *", instead
  // of a "const char * const *", which is only allowed in C++. fts_open()
  // does not modify the argument.
  std::string p(path);
  char *files[] = {const_cast<char *>(p.c_str()), NULL};

  // FTS_NOCHDIR  - Avoid changing cwd, which could cause unexpected behavior
  //                in multithreaded programs
  // FTS_PHYSICAL - Don't follow symlinks. Prevents deletion of files outside
  //                of the specified directory
  // FTS_XDEV     - Don't cross filesystem boundaries
  ftsp = fts_open(files, FTS_NOCHDIR | FTS_PHYSICAL | FTS_XDEV, NULL);
  if (!ftsp) {
    return;
  }

  while ((curr = fts_read(ftsp))) {
#if __has_feature(memory_sanitizer)
    // fts_read doesn't have propper msan interceptors.  Unpoison it ourselves.
    if (curr) {
      __msan_unpoison(curr, sizeof(*curr));
      __msan_unpoison_string(curr->fts_accpath);
      __msan_unpoison_string(curr->fts_path);
      __msan_unpoison_string(curr->fts_name);
    }
#endif
    switch (curr->fts_info) {
      case FTS_NS:
      case FTS_DNR:
      case FTS_ERR:
        LOG(WARNING) << "Can't read " << curr->fts_accpath;
        break;

      case FTS_DC:
      case FTS_DOT:
      case FTS_NSOK:
        // Not reached unless FTS_LOGICAL, FTS_SEEDOT, or FTS_NOSTAT were
        // passed to fts_open()
        break;

      case FTS_D:
        // Do nothing. Need depth-first search, so directories are deleted
        // in FTS_DP
        break;

      case FTS_DP:
      case FTS_F:
      case FTS_SL:
      case FTS_SLNONE:
      case FTS_DEFAULT:
        VLOG(1) << "Removing " << curr->fts_path;
        if (remove(curr->fts_accpath) < 0) {
          LOG(WARNING) << curr->fts_path
                       << ": Failed to remove: " << strerror(curr->fts_errno);
        }
        break;
    }
  }

  if (ftsp) {
    fts_close(ftsp);
  }
}

std::shared_ptr<absl::Span<uint8_t>> MMapFile(const std::string &path,
                                              FileOptions options) {
  int fd =
      open(path.c_str(), options == FileOptions::kReadable ? O_RDONLY : O_RDWR);
  PCHECK(fd != -1) << "Unable to open file " << path;
  struct stat sb;
  PCHECK(fstat(fd, &sb) != -1) << ": Unable to get file size of " << path;
  uint8_t *start = reinterpret_cast<uint8_t *>(mmap(
      NULL, sb.st_size,
      options == FileOptions::kReadable ? PROT_READ : (PROT_READ | PROT_WRITE),
      MAP_SHARED, fd, 0));
  CHECK(start != MAP_FAILED) << ": Unable to open mapping to file " << path;
  std::shared_ptr<absl::Span<uint8_t>> span =
      std::shared_ptr<absl::Span<uint8_t>>(
          new absl::Span<uint8_t>(start, sb.st_size),
          [](absl::Span<uint8_t> *span) {
            PCHECK(msync(span->data(), span->size(), MS_SYNC) == 0)
                << ": Failed to flush data before unmapping.";
            PCHECK(munmap(span->data(), span->size()) != -1);
            delete span;
          });
  close(fd);
  return span;
}

}  // namespace util
}  // namespace aos
