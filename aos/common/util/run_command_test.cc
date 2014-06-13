#include "aos/common/util/run_command.h"

#include "gtest/gtest.h"

#include "aos/common/util/thread.h"

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

TEST(RunCommandTest, MultipleThreads) {
  int result1, result2;
  util::FunctionThread t1([&result1](util::Thread *) {
    result1 = RunCommand("true");
  });
  util::FunctionThread t2([&result2](util::Thread *) {
    result2 = RunCommand("true");
  });
  t1.Start();
  t2.Start();
  t1.WaitUntilDone();
  t2.WaitUntilDone();
  ASSERT_NE(-1, result1);
  ASSERT_NE(-1, result2);
  ASSERT_TRUE(WIFEXITED(result1));
  ASSERT_TRUE(WIFEXITED(result2));
  EXPECT_EQ(0, WEXITSTATUS(result1));
  EXPECT_EQ(0, WEXITSTATUS(result2));
}

}  // namespace testing
}  // namespace util
}  // namespace aos
