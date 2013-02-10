#include <string>

#include "gtest/gtest.h"

#include "aos/aos_core.h"
#include "aos/atom_code/ipc_lib/sharedmem_test_setup.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/inttypes.h"
#include "aos/common/time.h"

using ::aos::time::Time;
using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;

namespace aos {
namespace testing {

static const std::string kLoggingName = "LoggingTestName";

class LoggingTest : public SharedMemTestSetup {
  virtual void SetUp() {
    SharedMemTestSetup::SetUp();
    ASSERT_EQ(0, log_init(kLoggingName.c_str()));
  }
  virtual void TearDown() {
    log_uninit();
    SharedMemTestSetup::TearDown();
  }

 public:
  AssertionResult WasAnythingLogged() {
    const log_message *msg = log_read_next1(NON_BLOCK);
    if (msg != NULL) {
      char bad_msg[LOG_MESSAGE_LEN];
      memcpy(bad_msg, msg->message, sizeof(bad_msg));
      log_free_message(msg);
      return AssertionSuccess() << "read message '" << bad_msg << "'";
    }
    return AssertionFailure();
  }
  AssertionResult WasLogged(log_level level, const std::string value) {
    const log_message *msg = NULL;
    char bad_msg[LOG_MESSAGE_LEN];
    bad_msg[0] = '\0';
    const pid_t owner = getpid();
    while (true) {
      if (msg != NULL) {
        static_assert(sizeof(bad_msg) == sizeof(msg->message),
                      "something is wrong");
        if (bad_msg[0] != '\0') {
          printf("read bad message: %s", bad_msg);
        }
        memcpy(bad_msg, msg->message, sizeof(bad_msg));
        log_free_message(msg);
        msg = NULL;
      }
      msg = log_read_next1(NON_BLOCK);
      if (msg == NULL) {
        return AssertionFailure() << "last message read was '" << bad_msg << "'"
            " instead of '" << value << "'";
      }
      if (msg->source != owner) continue;
      if (msg->level != level) continue;
      if (strcmp(msg->name, kLoggingName.c_str()) != 0) continue;
      if (strcmp(msg->message + strlen(msg->message) - value.length(),
                 value.c_str()) != 0) continue;

      // if it's gotten this far, then the message is correct
      log_free_message(msg);
      return AssertionSuccess();
    }
  }
};
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
  char *expected;
  ASSERT_GT(asprintf(&expected, "atom_logging_test.cpp: %d-%d: "
                        "first part second part (=19) third part last part 5\n",
                        begin_line + 1, end_line + 1), 0);
  EXPECT_TRUE(WasLogged(WARNING, std::string(expected)));
}

TEST_F(LoggingDeathTest, Fatal) {
  ASSERT_EXIT(LOG(FATAL, "this should crash it\n"),
              ::testing::KilledBySignal(SIGABRT),
              "this should crash it");
}

TEST_F(LoggingTest, PrintfDirectives) {
  LOG(INFO, "test log %%1 %%d\n");
  EXPECT_TRUE(WasLogged(INFO, "test log %1 %d\n"));
  LOG_DYNAMIC(WARNING, "test log %%2 %%f\n");
  EXPECT_TRUE(WasLogged(WARNING, "test log %2 %f\n"));
  LOG_CORK("log 3 part %%1 %%d ");
  LOG_UNCORK(DEBUG, "log 3 part %%2 %%f\n");
  EXPECT_TRUE(WasLogged(DEBUG, "log 3 part %1 %d log 3 part %2 %f\n"));
}

// For writing only.
TEST_F(LoggingTest, Timing) {
  static const long kTimingCycles = 5000;
  Time start = Time::Now();
  for (long i = 0; i < kTimingCycles; ++i) {
    LOG(INFO, "a\n");
  }
  Time elapsed = Time::Now() - start;

  printf("short message took %"PRId64" nsec for %ld\n", elapsed.ToNSec(),
      kTimingCycles);

  start = Time::Now();
  for (long i = 0; i < kTimingCycles; ++i) {
    LOG(INFO, "something longer than just \"a\" to log to test timing\n");
  }
  elapsed = Time::Now() - start;
  printf("long message took %"PRId64" nsec for %ld\n", elapsed.ToNSec(),
      kTimingCycles);
}

}  // namespace testing
}  // namespace aos
