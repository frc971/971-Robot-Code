#ifndef AOS_UTIL_FILE_H_
#define AOS_UTIL_FILE_H_

#include <string>

#include "absl/strings/string_view.h"

namespace aos {
namespace util {

// Returns the complete contents of filename. LOG(FATAL)s if any errors are
// encountered.
::std::string ReadFileToStringOrDie(const absl::string_view filename);

}  // namespace util
}  // namespace aos

#endif  // AOS_UTIL_FILE_H_
