#ifndef AOS_COMMON_UTIL_FILE_H_
#define AOS_COMMON_UTIL_FILE_H_

#include <string>

namespace aos {
namespace util {

// Returns the complete contents of filename. LOG(FATAL)s if any errors are
// encountered.
::std::string ReadFileToStringOrDie(const ::std::string &filename);

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_FILE_H_
