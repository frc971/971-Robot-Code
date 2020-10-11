#ifndef AOS_TESTING_TMPDIR_H_
#define AOS_TESTING_TMPDIR_H_

#include <string>

namespace aos {
namespace testing {

// Returns a usable temporary directory.
std::string TestTmpDir();

}  // namespace testing
}  // namespace aos

#endif  // AOS_TESTING_TMPDIR_H_
