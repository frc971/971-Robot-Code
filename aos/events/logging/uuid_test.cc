#include "aos/events/logging/uuid.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

// Tests that random UUIDs are actually random, and we can convert them to a
// string.  Not very exhaustive, but it is a good smoke test.
TEST(UUIDTest, GetOne) {
  LOG(INFO) << UUID::Random().string_view();

  EXPECT_NE(UUID::Random(), UUID::Random());
  EXPECT_NE(UUID::Random(), UUID::Zero());
  EXPECT_EQ(UUID::Zero(), UUID::Zero());
}

}  // namespace testing
}  // namespace aos
