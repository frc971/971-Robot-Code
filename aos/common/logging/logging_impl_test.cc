#include <string>

#include "gtest/gtest.h"

#include "aos/common/logging/logging_impl.h"
#include "aos/common/time.h"
#include "aos/common/die.h"
#include <inttypes.h>

using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;

namespace aos {
namespace logging {
namespace testing {

class TestLogImplementation : public LogImplementation {
  virtual void DoLog(log_level level, const char *format, va_list ap) {
    internal::FillInMessage(level, format, ap, &message_);

    used_ = true;
  }

  LogMessage message_;

 public:
  const LogMessage &message() { return message_; }
  bool used() { return used_; }
  void reset_used() { used_ = false; }

  TestLogImplementation() : used_(false) {}

  bool used_;
};
class LoggingTest : public ::testing::Test {
 protected:
  AssertionResult WasAnythingLogged() {
    if (log_implementation->used()) {
      return AssertionSuccess() << "read message '" <<
          log_implementation->message().message << "'";
    }
    return AssertionFailure();
  }
  AssertionResult WasLogged(log_level level, const std::string message) {
    if (!log_implementation->used()) {
      return AssertionFailure() << "nothing was logged";
    }
    if (log_implementation->message().level != level) {
      return AssertionFailure() << "a message with level " <<
          log_str(log_implementation->message().level) <<
          " was logged instead of " << log_str(level);
    }
    internal::Context *context = internal::Context::Get();
    if (log_implementation->message().source != context->source) {
      Die("got a message from %" PRIu32 ", but we're %" PRIu32 "\n",
          static_cast<uint32_t>(log_implementation->message().source),
          static_cast<uint32_t>(context->source));
    }
    if (strcmp(log_implementation->message().name,
               context->name.c_str()) != 0) {
      Die("got a message from %s, but we're %s\n",
          log_implementation->message().name, context->name.c_str());
    }
    if (strstr(log_implementation->message().message, message.c_str())
        == NULL) {
      return AssertionFailure() << "got a message of '" <<
          log_implementation->message().message <<
          "' but expected it to contain '" << message << "'";
    }

    return AssertionSuccess() << log_implementation->message().message;
  }

 private:
  virtual void SetUp() {
    static bool first = true;
    if (first) {
      first = false;

      Init();
      // We'll end up with several of them stacked up, but it really doesn't
      // matter.
      AddImplementation(log_implementation = new TestLogImplementation());
    }

    log_implementation->reset_used();
  }
  virtual void TearDown() {
    Cleanup();
  }

  static TestLogImplementation *log_implementation;
};
TestLogImplementation *LoggingTest::log_implementation(NULL);
typedef LoggingTest LoggingDeathTest;

// Tests both basic logging functionality and that the test setup works
// correctly.
TEST_F(LoggingTest, Basic) {
  EXPECT_FALSE(WasAnythingLogged());
  LOG(INFO, "test log 1\n");
  EXPECT_TRUE(WasLogged(INFO, "test log 1\n"));
  LOG(INFO, "test log 1.5\n");
  // there's a subtle typo on purpose...
  EXPECT_FALSE(WasLogged(INFO, "test log 15\n"));
  LOG(ERROR, "test log 2=%d\n", 55);
  EXPECT_TRUE(WasLogged(ERROR, "test log 2=55\n"));
  LOG(ERROR, "test log 3\n");
  EXPECT_FALSE(WasLogged(WARNING, "test log 3\n"));
  LOG(WARNING, "test log 4\n");
  EXPECT_TRUE(WasAnythingLogged());
}
TEST_F(LoggingTest, Cork) {
  static const int begin_line = __LINE__;
  LOG_CORK("first part ");
  LOG_CORK("second part (=%d) ", 19);
  EXPECT_FALSE(WasAnythingLogged());
  LOG_CORK("third part ");
  static const int end_line = __LINE__;
  LOG_UNCORK(WARNING, "last part %d\n", 5);
  std::stringstream expected;
  expected << "logging_impl_test.cc: ";
  expected << (begin_line + 1);
  expected << "-";
  expected << (end_line + 1);
  expected << ": ";
  expected << __func__;
  expected << ": first part second part (=19) third part last part 5\n";
  EXPECT_TRUE(WasLogged(WARNING, expected.str()));
}

#ifndef __VXWORKS__
TEST_F(LoggingDeathTest, Fatal) {
  ASSERT_EXIT(LOG(FATAL, "this should crash it\n"),
              ::testing::KilledBySignal(SIGABRT),
              "this should crash it");
}
#endif

TEST_F(LoggingTest, PrintfDirectives) {
  LOG(INFO, "test log %%1 %%d\n");
  EXPECT_TRUE(WasLogged(INFO, "test log %1 %d\n"));
  LOG_DYNAMIC(WARNING, "test log %%2 %%f\n");
  EXPECT_TRUE(WasLogged(WARNING, "test log %2 %f\n"));
  LOG_CORK("log 3 part %%1 %%d ");
  LOG_UNCORK(DEBUG, "log 3 part %%2 %%f\n");
  EXPECT_TRUE(WasLogged(DEBUG, "log 3 part %1 %d log 3 part %2 %f\n"));
}

TEST_F(LoggingTest, Timing) {
  // For writing only.
  //static const long kTimingCycles = 5000000;
  static const long kTimingCycles = 5000;

  time::Time start = time::Time::Now();
  for (long i = 0; i < kTimingCycles; ++i) {
    LOG(INFO, "a\n");
  }
  time::Time end = time::Time::Now();
  time::Time diff = end - start;
  printf("short message took %" PRId64 " nsec for %ld\n",
         diff.ToNSec(), kTimingCycles);

  start = time::Time::Now();
  for (long i = 0; i < kTimingCycles; ++i) {
    LOG(INFO, "something longer than just \"a\" to log to test timing\n");
  }
  end = time::Time::Now();
  diff = end - start;
  printf("long message took %" PRId64 " nsec for %ld\n",
         diff.ToNSec(), kTimingCycles);
}

}  // namespace testing
}  // namespace logging
}  // namespace aos
