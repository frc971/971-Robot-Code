#include "aos/testing/tmpdir.h"

#include <cstdlib>
#include <string>

#include "aos/ipc_lib/shm_base.h"

namespace aos {
namespace testing {

namespace {
std::string TestTmpDirOr(std::string fallback) {
  const char *tmp_dir = std::getenv("TEST_TMPDIR");
  if (tmp_dir != nullptr) {
    return tmp_dir;
  }
  return fallback;
}
}  // namespace

std::string TestTmpDir() { return TestTmpDirOr("/tmp"); }

void SetTestShmBase() { SetShmBase(TestTmpDirOr(FLAGS_shm_base)); }

}  // namespace testing
}  // namespace aos
