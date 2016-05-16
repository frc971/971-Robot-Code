#include "aos/testing/random_seed.h"

#include <stdlib.h>

namespace aos {
namespace testing {

int RandomSeed() {
  const char *from_environment = getenv("TEST_RANDOM_SEED");
  if (from_environment != nullptr) {
    return atoi(from_environment);
  }
  return 1;
}

}  // namespace testing
}  // namespace aos
