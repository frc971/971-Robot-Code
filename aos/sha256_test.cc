#include "aos/sha256.h"

#include "gtest/gtest.h"

#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos::testing {

constexpr const char *kTestString = "Test String";
constexpr const char *kTestStringSha =
    "30c6ff7a44f7035af933babaea771bf177fc38f06482ad06434cbcc04de7ac14";

TEST(Sha256Test, ChecksumString) {
  EXPECT_EQ(kTestStringSha, Sha256(kTestString));
  EXPECT_EQ("2b4da12a4bfe66a061c24440521a9e5b994e4f0bcc47a17436275f5283bb6852",
            Sha256("Test String 2"));
}

TEST(Sha256Test, ChecksumFile) {
  const std::filesystem::path test_file =
      aos::testing::TestTmpDir() + "/test.txt";
  util::WriteStringToFileOrDie(test_file.string(), kTestString);
  EXPECT_EQ(kTestStringSha, Sha256OfFile(test_file));
}

}  // namespace aos::testing
