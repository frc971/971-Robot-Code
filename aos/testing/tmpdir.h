#ifndef AOS_TESTING_TMPDIR_H_
#define AOS_TESTING_TMPDIR_H_

#include <string>

namespace aos {
namespace testing {

// Returns a usable temporary directory.
std::string TestTmpDir();

// Sets shm_base to a folder inside of TEST_TMPDIR if set, or --shm_base
// otherwise.
void SetTestShmBase();

}  // namespace testing
}  // namespace aos

#endif  // AOS_TESTING_TMPDIR_H_
