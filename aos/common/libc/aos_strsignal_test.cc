#include "aos/common/libc/aos_strsignal.h"

#include <signal.h>

#include "gtest/gtest.h"

namespace aos {
namespace libc {
namespace testing {

// Tries a couple of easy ones.
TEST(StrsignalTest, Basic) {
  EXPECT_STREQ("Hangup", aos_strsignal(SIGHUP));
  EXPECT_STREQ("Broken pipe", aos_strsignal(SIGPIPE));
  EXPECT_STREQ("Real-time signal 2", aos_strsignal(SIGRTMIN + 2));
  EXPECT_STREQ("Unknown signal 155", aos_strsignal(155));
}

// Tests that all the signals give the same result as strsignal(3).
TEST(StrsignalTest, All) {
  for (int i = 0; i < SIGRTMAX + 5; ++i) {
    EXPECT_STREQ(strsignal(i), aos_strsignal(i));
  }
}

}  // namespace testing
}  // namespace libc
}  // namespace aos
