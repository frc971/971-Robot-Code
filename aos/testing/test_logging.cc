#include "aos/testing/test_logging.h"

#include <stdio.h>

#include <vector>

#include "gtest/gtest.h"

#include "absl/base/call_once.h"

#include "aos/logging/implementations.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/thread_local.h"

using ::aos::logging::LogMessage;

namespace aos {
namespace testing {
namespace {

class TestLogImplementation : public logging::HandleMessageLogImplementation {
 public:
  const ::std::vector<LogMessage> &messages() { return messages_; }

  ::aos::monotonic_clock::time_point monotonic_now() const override {
    return ::aos::monotonic_clock::now();
  }

  // This class has to be a singleton so that everybody can get access to the
  // same instance to read out the messages etc.
  static std::shared_ptr<TestLogImplementation> GetInstance() {
    static std::shared_ptr<TestLogImplementation> instance =
        std::make_unique<TestLogImplementation>();
    return instance;
  }

  // Clears out all of the messages already recorded.
  void ClearMessages() {
    std::unique_lock<aos::stl_mutex> locker(messages_mutex_);
    messages_.clear();
  }

  // Prints out all of the messages (like when a test fails).
  void PrintAllMessages() {
    std::unique_lock<aos::stl_mutex> locker(messages_mutex_);
    for (auto it = messages_.begin(); it != messages_.end(); ++it) {
      logging::internal::PrintMessage(stdout, *it);
    }
  }

  void SetOutputFile(const char *filename) {
    if (strcmp("-", filename) != 0) {
      FILE *newfile = fopen(filename, "w");

      if (newfile) {
        output_file_ = newfile;
      }
    }
  }

  void PrintMessagesAsTheyComeIn() { print_as_messages_come_in_ = true; }

  // Don't call these from outside this class.
  ~TestLogImplementation() {
    if (output_file_ != stdout) {
      fclose(output_file_);
    }
  }

 private:
  virtual void HandleMessage(const LogMessage &message) override {
    std::unique_lock<aos::stl_mutex> locker(messages_mutex_);
    if (message.level == FATAL || print_as_messages_come_in_) {
      logging::internal::PrintMessage(output_file_, message);
    }

    messages_.push_back(message);
  }

  ::std::vector<LogMessage> messages_;
  bool print_as_messages_come_in_ = false;
  FILE *output_file_ = stdout;
  aos::stl_mutex messages_mutex_;
};

class MyTestEventListener : public ::testing::EmptyTestEventListener {
  virtual void OnTestStart(const ::testing::TestInfo & /*test_info*/) {
    TestLogImplementation::GetInstance()->ClearMessages();
  }
  virtual void OnTestEnd(const ::testing::TestInfo &test_info) {
    if (test_info.result()->Failed()) {
      printf("Test %s failed. Use '--print-logs' to see all log messages.\n",
             test_info.name());
    }
  }

  virtual void OnTestPartResult(const ::testing::TestPartResult &result) {
    if (result.failed()) {
      const char *failure_type = "unknown";
      switch (result.type()) {
        case ::testing::TestPartResult::Type::kNonFatalFailure:
          failure_type = "EXPECT";
          break;
        case ::testing::TestPartResult::Type::kFatalFailure:
          failure_type = "ASSERT";
          break;
        case ::testing::TestPartResult::Type::kSuccess:
        case ::testing::TestPartResult::Type::kSkip:
          break;
      }
      log_do(ERROR, "%s: %d: gtest %s failure\n%s\n", result.file_name(),
             result.line_number(), failure_type, result.message());
    }
  }
};

void *DoEnableTestLogging() {
  logging::SetImplementation(TestLogImplementation::GetInstance());

  ::testing::UnitTest::GetInstance()->listeners().Append(
      new MyTestEventListener());

  return nullptr;
}

static absl::once_flag enable_test_logging_once;

}  // namespace

void EnableTestLogging() {
  absl::call_once(enable_test_logging_once, DoEnableTestLogging);
}

void SetLogFileName(const char *filename) {
  TestLogImplementation::GetInstance()->SetOutputFile(filename);
}

void ForcePrintLogsDuringTests() {
  TestLogImplementation::GetInstance()->PrintMessagesAsTheyComeIn();
}

}  // namespace testing
}  // namespace aos
