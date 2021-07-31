#include "aos/util/file.h"

#include <cstdlib>
#include <string>

#include "gtest/gtest.h"

namespace aos {
namespace util {
namespace testing {

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFile) {
  const ::std::string tmpdir(getenv("TEST_TMPDIR"));
  const ::std::string test_file = tmpdir + "/test_file";
  ASSERT_EQ(0, system(("echo contents > " + test_file).c_str()));
  EXPECT_EQ("contents\n", ReadFileToStringOrDie(test_file));
}

// Tests reading a file with 0 size, among other weird things.
TEST(FileTest, ReadSpecialFile) {
  const ::std::string stat = ReadFileToStringOrDie("/proc/self/stat");
  EXPECT_EQ('\n', stat[stat.size() - 1]);
  const ::std::string my_pid = ::std::to_string(getpid());
  EXPECT_EQ(my_pid, stat.substr(0, my_pid.size()));
}

// Tests that the PathExists function works under normal conditions.
TEST(FileTest, PathExistsTest) {
  const std::string tmpdir(getenv("TEST_TMPDIR"));
  const std::string test_file = tmpdir + "/test_file";
  // Make sure the test_file doesn't exist.
  unlink(test_file.c_str());
  EXPECT_FALSE(PathExists(test_file));

  WriteStringToFileOrDie(test_file, "abc");

  EXPECT_TRUE(PathExists(test_file));
}

}  // namespace testing
}  // namespace util
}  // namespace aos
