#include "aos/testing/test_logging.h"

#include <stdio.h>

#include <vector>

#include "gtest/gtest.h"

#include "aos/logging/implementations.h"
#include "aos/mutex/mutex.h"
#include "aos/once.h"

using ::aos::logging::LogMessage;

namespace aos {
namespace testing {
namespace {

class TestLogImplementation : public logging::HandleMessageLogImplementation {
 public:
  const ::std::vector<LogMessage> &messages() { return messages_; }

  // Sets the current thread's time to be monotonic_now for logging.
  void MockTime(::aos::monotonic_clock::time_point monotonic_now) {
    mock_time_ = true;
    monotonic_now_ = monotonic_now;
  }

  // Clears any mock time for the current thread.
  void UnMockTime() { mock_time_ = false; }

  ::aos::monotonic_clock::time_point monotonic_now() const override {
    if (mock_time_) {
      return monotonic_now_;
    }
    return ::aos::monotonic_clock::now();
  }

  // This class has to be a singleton so that everybody can get access to the
  // same instance to read out the messages etc.
  static TestLogImplementation *GetInstance() {
    static Once<TestLogImplementation> once(CreateInstance);
    return once.Get();
  }

  // Clears out all of the messages already recorded.
  void ClearMessages() {
    ::aos::MutexLocker locker(&messages_mutex_);
    messages_.clear();
  }

  // Prints out all of the messages (like when a test fails).
  void PrintAllMessages() {
    ::aos::MutexLocker locker(&messages_mutex_);
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

  bool fill_type_cache() override { return false; }

  void PrintMessagesAsTheyComeIn() { print_as_messages_come_in_ = true; }

 private:
  TestLogImplementation() {}
  ~TestLogImplementation() {
    if (output_file_ != stdout) {
      fclose(output_file_);
    }
  }

  static TestLogImplementation *CreateInstance() {
    return new TestLogImplementation();
  }

  virtual void HandleMessage(const LogMessage &message) override {
    ::aos::MutexLocker locker(&messages_mutex_);
    if (message.level == FATAL || print_as_messages_come_in_) {
      logging::internal::PrintMessage(output_file_, message);
    }

    messages_.push_back(message);
  }

  ::std::vector<LogMessage> messages_;
  bool print_as_messages_come_in_ = false;
  FILE *output_file_ = stdout;
  ::aos::Mutex messages_mutex_;

  // Thread local storage for mock time.  This is thread local because if
  // someone spawns a thread and goes to town in parallel with a simulated event
  // loop, we want to just print the actual monotonic clock out.
  static thread_local bool mock_time_;
  static thread_local ::aos::monotonic_clock::time_point monotonic_now_;
};

thread_local bool TestLogImplementation::mock_time_ = false;
thread_local ::aos::monotonic_clock::time_point
    TestLogImplementation::monotonic_now_ = ::aos::monotonic_clock::min_time;

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

  virtual void OnTestPartResult( const ::testing::TestPartResult &result) {
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
          break;
      }
      log_do(ERROR, "%s: %d: gtest %s failure\n%s\n",
             result.file_name(),
             result.line_number(),
             failure_type,
             result.message());
    }
  }
};

void *DoEnableTestLogging() {
  logging::Init();
  logging::AddImplementation(TestLogImplementation::GetInstance());

  ::testing::UnitTest::GetInstance()->listeners().Append(
      new MyTestEventListener());

  return nullptr;
}

Once<void> enable_test_logging_once(DoEnableTestLogging);

}  // namespace

void EnableTestLogging() {
  enable_test_logging_once.Get();
}

void SetLogFileName(const char* filename) {
  TestLogImplementation::GetInstance()->SetOutputFile(filename);
}

void ForcePrintLogsDuringTests() {
  TestLogImplementation::GetInstance()->PrintMessagesAsTheyComeIn();
}

void MockTime(::aos::monotonic_clock::time_point monotonic_now) {
  TestLogImplementation::GetInstance()->MockTime(monotonic_now);
}
void UnMockTime() {
  TestLogImplementation::GetInstance()->UnMockTime();
}

}  // namespace testing
}  // namespace aos
