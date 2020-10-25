#include "aos/logging/implementations.h"

#include <inttypes.h>

#include <chrono>
#include <string>

#include "aos/logging/printf_formats.h"
#include "aos/time/time.h"
#include "gtest/gtest.h"

using ::testing::AssertionFailure;
using ::testing::AssertionResult;
using ::testing::AssertionSuccess;

namespace aos {
namespace logging {
namespace testing {

namespace chrono = ::std::chrono;

class TestLogImplementation : public LogImplementation {
  virtual ::aos::monotonic_clock::time_point monotonic_now() const {
    return ::aos::monotonic_clock::now();
  }

  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0))) void DoLog(
      log_level level, const char *format, va_list ap) override {
    internal::FillInMessage(level, monotonic_now(), format, ap, &message_);

    if (level == FATAL) {
      internal::PrintMessage(stderr, message_);
      abort();
    }

    used_ = true;
  }

  LogMessage message_;

 public:
  const LogMessage &message() { return message_; }
  bool used() { return used_; }

  bool used_ = false;
};
class LoggingTest : public ::testing::Test {
 protected:
  AssertionResult WasAnythingLogged() {
    if (log_implementation->used()) {
      return AssertionSuccess() << "read message '"
                                << log_implementation->message().message << "'";
    }
    return AssertionFailure();
  }
  AssertionResult WasLogged(log_level level, const std::string message) {
    if (!log_implementation->used()) {
      return AssertionFailure() << "nothing was logged";
    }
    if (log_implementation->message().level != level) {
      return AssertionFailure() << "a message with level "
                                << log_str(log_implementation->message().level)
                                << " was logged instead of " << log_str(level);
    }
    internal::Context *context = internal::Context::Get();
    if (log_implementation->message().source != context->source) {
      AOS_LOG(FATAL, "got a message from %" PRIu32 ", but we're %" PRIu32 "\n",
              static_cast<uint32_t>(log_implementation->message().source),
              static_cast<uint32_t>(context->source));
    }
    if (log_implementation->message().name_length != context->name_size ||
        memcmp(log_implementation->message().name, context->name,
               context->name_size) != 0) {
      AOS_LOG(FATAL, "got a message from %.*s, but we're %s\n",
              static_cast<int>(log_implementation->message().name_length),
              log_implementation->message().name, context->name);
    }
    if (strstr(log_implementation->message().message, message.c_str()) ==
        NULL) {
      return AssertionFailure()
             << "got a message of '" << log_implementation->message().message
             << "' but expected it to contain '" << message << "'";
    }

    return AssertionSuccess() << log_implementation->message().message;
  }

 private:
  void SetUp() override {
    log_implementation = std::make_shared<TestLogImplementation>();
    SetImplementation(log_implementation);
  }
  void TearDown() override {
    SetImplementation(nullptr);
    internal::Context::DeleteNow();
    CHECK_EQ(log_implementation.use_count(), 1);
    log_implementation.reset();
  }

  std::shared_ptr<TestLogImplementation> log_implementation;
};
typedef LoggingTest LoggingDeathTest;

// Tests both basic logging functionality and that the test setup works
// correctly.
TEST_F(LoggingTest, Basic) {
  EXPECT_FALSE(WasAnythingLogged());
  AOS_LOG(INFO, "test log 1\n");
  EXPECT_TRUE(WasLogged(INFO, "test log 1\n"));
  AOS_LOG(INFO, "test log 1.5\n");
  // there's a subtle typo on purpose...
  EXPECT_FALSE(WasLogged(INFO, "test log 15\n"));
  AOS_LOG(ERROR, "test log 2=%d\n", 55);
  EXPECT_TRUE(WasLogged(ERROR, "test log 2=55\n"));
  AOS_LOG(ERROR, "test log 3\n");
  EXPECT_FALSE(WasLogged(WARNING, "test log 3\n"));
  AOS_LOG(WARNING, "test log 4\n");
  EXPECT_TRUE(WasAnythingLogged());
}

TEST_F(LoggingDeathTest, Fatal) {
  ASSERT_EXIT(AOS_LOG(FATAL, "this should crash it\n"),
              ::testing::KilledBySignal(SIGABRT), "this should crash it");
}

TEST_F(LoggingDeathTest, PCHECK) {
  EXPECT_DEATH(AOS_PCHECK(fprintf(stdin, "nope")),
               ".*fprintf\\(stdin, \"nope\"\\).*failed.*");
}

TEST_F(LoggingTest, PCHECK) { EXPECT_EQ(7, AOS_PCHECK(printf("abc123\n"))); }

TEST_F(LoggingTest, PrintfDirectives) {
  AOS_LOG(INFO, "test log %%1 %%d\n");
  EXPECT_TRUE(WasLogged(INFO, "test log %1 %d\n"));
  AOS_LOG_DYNAMIC(WARNING, "test log %%2 %%f\n");
  EXPECT_TRUE(WasLogged(WARNING, "test log %2 %f\n"));
}

TEST_F(LoggingTest, Timing) {
  // For writing only.
  // static const long kTimingCycles = 5000000;
  static const long kTimingCycles = 5000;

  monotonic_clock::time_point start = monotonic_clock::now();
  for (long i = 0; i < kTimingCycles; ++i) {
    AOS_LOG(INFO, "a\n");
  }
  monotonic_clock::time_point end = monotonic_clock::now();
  auto diff = end - start;
  printf("short message took %" PRId64 " nsec for %ld\n",
         static_cast<int64_t>(
             chrono::duration_cast<chrono::nanoseconds>(diff).count()),
         kTimingCycles);

  start = monotonic_clock::now();
  for (long i = 0; i < kTimingCycles; ++i) {
    AOS_LOG(INFO, "something longer than just \"a\" to log to test timing\n");
  }
  end = monotonic_clock::now();
  diff = end - start;
  printf("long message took %" PRId64 " nsec for %ld\n",
         static_cast<int64_t>(
             chrono::duration_cast<chrono::nanoseconds>(diff).count()),
         kTimingCycles);
}

TEST(LoggingPrintFormatTest, Time) {
  char buffer[1024];

  // Easy ones.
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(2, 0)));
  EXPECT_EQ("0000000002.000000s", ::std::string(buffer));
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(2, 1)));
  EXPECT_EQ("0000000002.000000s", ::std::string(buffer));

  // This one should be exact.
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(2, 1000)));
  EXPECT_EQ("0000000002.000001s", ::std::string(buffer));

  // Make sure rounding works correctly.
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(2, 999)));
  EXPECT_EQ("0000000002.000000s", ::std::string(buffer));
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(2, 500)));
  EXPECT_EQ("0000000002.000000s", ::std::string(buffer));
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(2, 499)));
  EXPECT_EQ("0000000002.000000s", ::std::string(buffer));

  // This used to result in "0000000001.099500s".
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(1, 995000000)));
  EXPECT_EQ("0000000001.995000s", ::std::string(buffer));

  // This used to result in "0000000001.099500s".
  ASSERT_EQ(18, snprintf(buffer, sizeof(buffer), AOS_TIME_FORMAT,
                         AOS_TIME_ARGS(1, 999999999)));
  EXPECT_EQ("0000000001.999999s", ::std::string(buffer));
}

TEST(LoggingPrintFormatTest, Base) {
  char buffer[1024];

  static const ::std::string kExpected1 =
      "name(971)(01678): ERROR   at 0000000001.995000s: ";
  ASSERT_GT(sizeof(buffer), kExpected1.size());
  ASSERT_EQ(
      kExpected1.size(),
      static_cast<size_t>(snprintf(
          buffer, sizeof(buffer), AOS_LOGGING_BASE_FORMAT,
          AOS_LOGGING_BASE_ARGS(4, "name", 971, 1678, ERROR, 1, 995000000))));
  EXPECT_EQ(kExpected1, ::std::string(buffer));
}

TEST(ScopedLogRestorerTest, RestoreTest) {
  SetImplementation(std::make_shared<StreamLogImplementation>(stdout));
  LogImplementation *curr_impl = GetImplementation().get();

  {
    ScopedLogRestorer log_restorer;

    log_restorer.Swap(
        std::make_shared<CallbackLogImplementation>([](const LogMessage &) {}));
    ASSERT_NE(curr_impl, GetImplementation().get());
  }

  ASSERT_EQ(curr_impl, GetImplementation().get());
}

}  // namespace testing
}  // namespace logging
}  // namespace aos
