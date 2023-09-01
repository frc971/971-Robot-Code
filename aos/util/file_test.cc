#include "aos/util/file.h"

#include <cstdlib>
#include <optional>
#include <string>

#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

#include "aos/realtime.h"
#include "aos/testing/tmpdir.h"

namespace aos {
namespace util {
namespace testing {

using ::testing::ElementsAre;

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFile) {
  const std::string tmpdir(aos::testing::TestTmpDir());
  const std::string test_file = tmpdir + "/test_file";
  ASSERT_EQ(0, system(("echo contents > " + test_file).c_str()));
  EXPECT_EQ("contents\n", ReadFileToStringOrDie(test_file));
}

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFileToBytes) {
  const std::string tmpdir(aos::testing::TestTmpDir());
  const std::string test_file = tmpdir + "/test_file";
  ASSERT_EQ(0, system(("echo contents > " + test_file).c_str()));
  EXPECT_THAT(ReadFileToVecOrDie(test_file),
              ElementsAre('c', 'o', 'n', 't', 'e', 'n', 't', 's', '\n'));
}

// Tests reading a file with 0 size, among other weird things.
TEST(FileTest, ReadSpecialFile) {
  const std::string stat = ReadFileToStringOrDie("/proc/self/stat");
  EXPECT_EQ('\n', stat[stat.size() - 1]);
  const std::string my_pid = ::std::to_string(getpid());
  EXPECT_EQ(my_pid, stat.substr(0, my_pid.size()));
}

// Basic test of maybe reading a normal file.
TEST(FileTest, MaybeReadNormalFile) {
  const std::string tmpdir(aos::testing::TestTmpDir());
  const std::string test_file = tmpdir + "/test_file";
  ASSERT_EQ(0, system(("echo contents > " + test_file).c_str()));
  EXPECT_EQ("contents\n", MaybeReadFileToString(test_file).value());
}

// Tests maybe reading a file with 0 size, among other weird things.
TEST(FileTest, MaybeReadSpecialFile) {
  const std::optional<std::string> stat =
      MaybeReadFileToString("/proc/self/stat");
  ASSERT_TRUE(stat.has_value());
  EXPECT_EQ('\n', (*stat)[stat->size() - 1]);
  const std::string my_pid = std::to_string(getpid());
  EXPECT_EQ(my_pid, stat->substr(0, my_pid.size()));
}

// Tests maybe reading a non-existent file, and not fatally erroring.
TEST(FileTest, MaybeReadNonexistentFile) {
  const std::optional<std::string> contents = MaybeReadFileToString("/dne");
  ASSERT_FALSE(contents.has_value());
}

// Tests that the PathExists function works under normal conditions.
TEST(FileTest, PathExistsTest) {
  const std::string tmpdir(aos::testing::TestTmpDir());
  const std::string test_file = tmpdir + "/test_file";
  // Make sure the test_file doesn't exist.
  unlink(test_file.c_str());
  EXPECT_FALSE(PathExists(test_file));

  WriteStringToFileOrDie(test_file, "abc");

  EXPECT_TRUE(PathExists(test_file));
}

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFileNoMalloc) {
  const ::std::string tmpdir(aos::testing::TestTmpDir());
  const ::std::string test_file = tmpdir + "/test_file";
  // Make sure to include a string long enough to avoid small string
  // optimization.
  ASSERT_EQ(0, system(("echo 123456789 > " + test_file).c_str()));

  FileReader reader(test_file);

  aos::ScopedRealtime realtime;
  {
    std::array<char, 20> contents;
    std::optional<absl::Span<char>> read_result =
        reader.ReadContents({contents.data(), contents.size()});
    EXPECT_EQ("123456789\n",
              std::string_view(read_result->data(), read_result->size()));
  }
  {
    std::optional<std::array<char, 10>> read_result = reader.ReadString<10>();
    ASSERT_TRUE(read_result.has_value());
    EXPECT_EQ("123456789\n",
              std::string_view(read_result->data(), read_result->size()));
  }
  EXPECT_EQ(123456789, reader.ReadInt32());
}

// Tests that we can write to a file without malloc'ing.
TEST(FileTest, WriteNormalFileNoMalloc) {
  const ::std::string tmpdir(aos::testing::TestTmpDir());
  const ::std::string test_file = tmpdir + "/test_file";

  FileWriter writer(test_file);

  FileWriter::WriteResult result;
  {
    aos::ScopedRealtime realtime;
    result = writer.WriteBytes("123456789");
  }
  EXPECT_EQ(9, result.bytes_written);
  EXPECT_EQ(9, result.return_code);
  EXPECT_EQ("123456789", ReadFileToStringOrDie(test_file));
}

// Tests that if we fail to write a file that the error code propagates
// correctly.
TEST(FileTest, WriteFileError) {
  const ::std::string tmpdir(aos::testing::TestTmpDir());
  const ::std::string test_file = tmpdir + "/test_file";

  // Open with only read permissions; this should cause things to fail.
  FileWriter writer(test_file, S_IRUSR);

  // Mess up the file management by closing the file descriptor.
  PCHECK(0 == close(writer.fd()));

  FileWriter::WriteResult result;
  {
    aos::ScopedRealtime realtime;
    result = writer.WriteBytes("123456789");
  }
  EXPECT_EQ(0, result.bytes_written);
  EXPECT_EQ(-1, result.return_code);
  EXPECT_EQ("", ReadFileToStringOrDie(test_file));
}

}  // namespace testing
}  // namespace util
}  // namespace aos
