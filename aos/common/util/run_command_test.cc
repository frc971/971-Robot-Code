#include "aos/common/util/run_command.h"

#include "gtest/gtest.h"

namespace aos {
namespace util {
namespace testing {

TEST(RunCommandTest, True) {
  int result = RunCommand("true");
  ASSERT_NE(-1, result);
  ASSERT_TRUE(WIFEXITED(result));
  EXPECT_EQ(0, WEXITSTATUS(result));
}

TEST(RunCommandTest, False) {
  int result = RunCommand("false");
  ASSERT_NE(-1, result);
  ASSERT_TRUE(WIFEXITED(result));
  EXPECT_EQ(1, WEXITSTATUS(result));
}

TEST(RunCommandTest, CommandNotFound) {
  int result = RunCommand("ajflkjasdlfa");
  ASSERT_NE(-1, result);
  ASSERT_TRUE(WIFEXITED(result));
  EXPECT_EQ(127, WEXITSTATUS(result));
}

TEST(RunCommandTest, KilledBySignal) {
  int result = RunCommand("kill -QUIT $$");
  ASSERT_NE(-1, result);
  ASSERT_TRUE(WIFSIGNALED(result));
  EXPECT_EQ(SIGQUIT, WTERMSIG(result));
}

}  // namespace testing
}  // namespace util
}  // namespace aos
