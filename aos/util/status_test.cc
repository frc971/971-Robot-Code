#include "aos/util/status.h"

#include <filesystem>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/realtime.h"
#include "aos/testing/path.h"

DECLARE_bool(die_on_malloc);

namespace aos::testing {
class StatusTest : public ::testing::Test {
 protected:
  StatusTest() {}
};

// Tests that we can construct an "Ok" status and that it presents the correct
// interface.
TEST_F(StatusTest, Okay) {
  std::optional<Status> ok;
  {
    aos::ScopedRealtime realtime;
    ok = Status::Ok();
  }
  ASSERT_TRUE(ok.has_value());
  EXPECT_TRUE(ok->ok());
  EXPECT_EQ(0, ok->code());
  EXPECT_EQ("", ok->message());
  EXPECT_FALSE(ok->source_location().has_value());
  EXPECT_EQ(std::string("Status is okay with code of 0 and message: "),
            ok->ToString());
}

// Tests that we can construct an errored status in realtime code.
TEST_F(StatusTest, RealtimeError) {
  std::optional<Status> error;
  {
    aos::ScopedRealtime realtime;
    error = Status::Error("Hello, World!");
  }
  const int line = __LINE__ - 2;
  ASSERT_TRUE(error.has_value());
  EXPECT_FALSE(error->ok());
  EXPECT_NE(0, error->code());
  EXPECT_EQ(std::string("Hello, World!"), error->message());
  ASSERT_TRUE(error->source_location().has_value());
  EXPECT_EQ(
      std::string("status_test.cc"),
      std::filesystem::path(error->source_location()->file_name()).filename());
  EXPECT_EQ(
      std::string("virtual void "
                  "aos::testing::StatusTest_RealtimeError_Test::TestBody()"),
      error->source_location()->function_name());
  EXPECT_EQ(line, error->source_location()->line());
  EXPECT_LT(1, error->source_location()->column());
  EXPECT_THAT(
      error->ToString(),
      ::testing::HasSubstr(absl::StrFormat(
          "status_test.cc:%d in virtual void "
          "aos::testing::StatusTest_RealtimeError_Test::TestBody(): Status is "
          "errored with code of 1 and message: Hello, World!",
          line)));
}

// Malloc hooks don't work with asan/msan.
#if !__has_feature(address_sanitizer) && !__has_feature(memory_sanitizer)
// Tests that we do indeed malloc (and catch it) on an extra-long error message
// (this is mostly intended to ensure that the test setup is working correctly).
TEST(StatusDeathTest, BlowsUpOnRealtimeAllocation) {
  std::string message(" ", Status::kStaticMessageLength + 1);
  EXPECT_DEATH(
      {
        aos::ScopedRealtime realtime;
        aos::CheckRealtime();
        Status foo = Status::Error(message);
      },
      "Malloced");
}

#endif

// Tests that we can use arbitrarily-sized string literals for error messages.
TEST_F(StatusTest, StringLiteralError) {
  std::optional<Status> error;
  const char *message =
      "Hellllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll"
      "llllllllllllllloooooooooooooooooooooooooooooooooooooooooooo, "
      "World!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      "!!!!!!!!!!!!!!";
  ASSERT_LT(Status::kStaticMessageLength, strlen(message));
  {
    aos::ScopedRealtime realtime;
    error = Status::StringLiteralError(message);
  }
  ASSERT_TRUE(error.has_value());
  EXPECT_FALSE(error->ok());
  EXPECT_EQ(message, error->message());
  ASSERT_TRUE(error->source_location().has_value());
  EXPECT_EQ(
      std::string("status_test.cc"),
      std::filesystem::path(error->source_location()->file_name()).filename());
}

// Tests that the CheckExpected() call works as intended.
TEST(StatusDeathTest, CheckExpected) {
  tl::expected<int, Status> expected;
  expected.emplace(971);
  EXPECT_EQ(971, CheckExpected(expected))
      << "Should have gotten out the emplaced value on no error.";
  expected = Status::UnexpectedError("Hello, World!");
  EXPECT_DEATH(CheckExpected(expected), "Hello, World!")
      << "An error message including the error string should have been printed "
         "on death.";
  EXPECT_DEATH(CheckExpected<void>(Status::UnexpectedError("void expected")),
               "void expected")
      << "A void expected should work with CheckExpected().";
}
}  // namespace aos::testing
