#include "aos/testing/random_seed.h"

#include <cstdlib>

namespace aos::testing {

int RandomSeed() {
  const char *from_environment = getenv("TEST_RANDOM_SEED");
  if (from_environment != nullptr) {
    return atoi(from_environment);
  }
  return 1;
}

}  // namespace aos::testing
