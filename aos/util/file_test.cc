#include "aos/util/file.h"

#include <cstdlib>
#include <string>

#include "gtest/gtest.h"
#include "aos/realtime.h"

DECLARE_bool(die_on_malloc);

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

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFileNoMalloc) {
  const ::std::string tmpdir(getenv("TEST_TMPDIR"));
  const ::std::string test_file = tmpdir + "/test_file";
  ASSERT_EQ(0, system(("echo 971 > " + test_file).c_str()));

  FileReader reader(test_file);

  FLAGS_die_on_malloc = true;
  RegisterMallocHook();
  aos::ScopedRealtime realtime;
  EXPECT_EQ("971\n", reader.ReadContents());
  EXPECT_EQ(971, reader.ReadInt());
}

}  // namespace testing
}  // namespace util
}  // namespace aos
