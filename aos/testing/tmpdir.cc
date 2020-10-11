#include "aos/testing/tmpdir.h"

#include <cstdlib>
#include <string>

namespace aos {
namespace testing {

std::string TestTmpDir() {
  const char *tmp_dir = std::getenv("TEST_TMPDIR");
  if (tmp_dir != nullptr) {
    return tmp_dir;
  }
  return "/tmp";
}

}  // namespace testing
}  // namespace aos
