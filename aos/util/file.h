#ifndef AOS_UTIL_FILE_H_
#define AOS_UTIL_FILE_H_

#include <sys/stat.h>
#include <string>
#include <string_view>

#include "glog/logging.h"

namespace aos {
namespace util {

// Returns the complete contents of filename. LOG(FATAL)s if any errors are
// encountered.
::std::string ReadFileToStringOrDie(const std::string_view filename);

// Creates filename if it doesn't exist and sets the contents to contents.
void WriteStringToFileOrDie(const std::string_view filename,
                            const std::string_view contents,
                            mode_t permissions = S_IRWXU);

// Returns true if it succeeds or false if the filesystem is full.
bool MkdirPIfSpace(std::string_view path, mode_t mode);

inline void MkdirP(std::string_view path, mode_t mode) {
  CHECK(MkdirPIfSpace(path, mode));
}

bool PathExists(std::string_view path);

// Recursively removes everything in the provided path.  Ignores any errors it
// runs across.
void UnlinkRecursive(std::string_view path);

}  // namespace util
}  // namespace aos

#endif  // AOS_UTIL_FILE_H_
