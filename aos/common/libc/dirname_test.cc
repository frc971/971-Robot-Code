#include "aos/common/libc/dirname.h"

#include <libgen.h>

#include "gtest/gtest.h"

namespace aos {
namespace libc {
namespace testing {

// Tests the examples from the Linux man-pages release 3.44 dirname(3).
TEST(DirnameTest, ManPageExamples) {
  EXPECT_EQ("/usr", Dirname("/usr/lib"));
  EXPECT_EQ("/", Dirname("/usr/"));
  EXPECT_EQ(".", Dirname("usr"));
  EXPECT_EQ("/", Dirname("/"));
  EXPECT_EQ(".", Dirname("."));
  EXPECT_EQ(".", Dirname(".."));
}

// Tests that it handles multiple '/'s in a row correctly.
TEST(DirnameTest, MultipleSlashes) {
  EXPECT_EQ("//usr", Dirname("//usr//lib"));
  EXPECT_EQ("//usr/lib", Dirname("//usr/lib//bla"));
  EXPECT_EQ("/", Dirname("//usr//"));
  EXPECT_EQ(".", Dirname("usr//"));
  EXPECT_EQ("/", Dirname("//"));
  EXPECT_EQ(".", Dirname(".//"));
  EXPECT_EQ(".", Dirname("..//"));
}

TEST(DirnameTest, WeirdInputs) {
  EXPECT_EQ(".", Dirname(""));
  EXPECT_EQ(".", Dirname("..."));
}

// Runs through a bunch of randomly constructed pathnames and makes sure it
// gives the same result as dirname(3).
TEST(DirnameTest, Random) {
  static const char kTestBytes[] = "a0//.. ";
  static const size_t kTestBytesSize = sizeof(kTestBytes) - 1;
  static const size_t kTestPathSize = 6;

  ::std::string test_string(kTestPathSize, '\0');
  char test_path[kTestPathSize + 1];
  for (size_t i0 = 0; i0 < kTestBytesSize; ++i0) {
    test_string[0] = kTestBytes[i0];
    for (size_t i1 = 0; i1 < kTestBytesSize; ++i1) {
      // dirname(3) returns "//" in this case which is weird and our Dirname
      // doesn't.
      if (test_string[0] == '/' && kTestBytes[i1] == '/') continue;

      test_string[1] = kTestBytes[i1];
      for (size_t i2 = 0; i2 < kTestBytesSize; ++i2) {
        test_string[2] = kTestBytes[i2];
        for (size_t i3 = 0; i3 < kTestBytesSize; ++i3) {
          test_string[3] = kTestBytes[i3];
          for (size_t i4 = 0; i4 < kTestBytesSize; ++i4) {
            test_string[4] = kTestBytes[i4];
            for (size_t i5 = 0; i5 < kTestBytesSize; ++i5) {
              test_string[5] = kTestBytes[i5];

              memcpy(test_path, test_string.c_str(), kTestPathSize);
              test_path[kTestPathSize] = '\0';

              SCOPED_TRACE("path is '" + test_string + "'");
              EXPECT_EQ(::std::string(dirname(test_path)),
                        Dirname(test_string));
            }
          }
        }
      }
    }
  }
}

}  // namespace testing
}  // namespace libc
}  // namespace aos
