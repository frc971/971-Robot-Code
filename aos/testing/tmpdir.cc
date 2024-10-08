#include "aos/testing/tmpdir.h"

#include <cstdlib>
#include <string>

#include "absl/flags/flag.h"

#include "aos/ipc_lib/shm_base.h"

namespace aos::testing {

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

void SetTestShmBase() {
  SetShmBase(TestTmpDirOr(absl::GetFlag(FLAGS_shm_base)));
}

}  // namespace aos::testing
