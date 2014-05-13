#include "aos/common/libc/aos_strerror.h"

#include <errno.h>

#include "gtest/gtest.h"

namespace aos {
namespace libc {
namespace testing {

// Tries a couple of easy ones.
TEST(StrerrorTest, Basic) {
  EXPECT_STREQ("Argument list too long", aos_strerror(E2BIG));
  EXPECT_STREQ("Bad file descriptor", aos_strerror(EBADF));
  EXPECT_STREQ("Unknown error 4021", aos_strerror(4021));
}

// Runs through all errno values and makes sure it gives the same result as
// strerror(3).
TEST(StrerrorTest, All) {
  for (int i = 0; i < 4095; ++i) {
    SCOPED_TRACE("iteration " + ::std::to_string(i));
    EXPECT_STREQ(strerror(i), aos_strerror(i));
  }
}

}  // namespace testing
}  // namespace libc
}  // namespace aos
