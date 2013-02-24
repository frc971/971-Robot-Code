#include "aos/common/queue_testutils.h"

#include <string.h>

#include "gtest/gtest.h"

#include "aos/common/queue.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/once.h"

using ::aos::logging::LogMessage;

namespace aos {
namespace common {
namespace testing {
namespace {

class TestLogImplementation : public logging::LogImplementation {
 public:
  const ::std::vector<LogMessage> &messages() { return messages_; }

  // This class has to be a singleton so that everybody can get access to the
  // same instance to read out the messages etc.
  static TestLogImplementation *GetInstance() {
    static Once<TestLogImplementation> once(CreateInstance);
    return once.Get();
  }

  // Clears out all of the messages already recorded.
  void ClearMessages() {
    messages_.clear();
  }

  // Prints out all of the messages (like when a test fails).
  void PrintAllMessages() {
    for (auto it = messages_.begin(); it != messages_.end(); ++it) {
      logging::internal::PrintMessage(stdout, *it);
    }
  }

 private:
  TestLogImplementation() {}

  static TestLogImplementation *CreateInstance() {
    return new TestLogImplementation();
  }

  virtual void DoLog(log_level level, const char *format, va_list ap) {
    LogMessage message;

    logging::internal::FillInMessage(level, format, ap, &message);

    if (!logging::log_gt_important(WARNING, level)) {
      logging::internal::PrintMessage(stdout, message);
    }

    messages_.push_back(message);
  }

  ::std::vector<LogMessage> messages_;
};

class MyTestEventListener : public ::testing::EmptyTestEventListener {
  virtual void OnTestStart(const ::testing::TestInfo &/*test_info*/) {
    TestLogImplementation::GetInstance()->ClearMessages();
  }
  virtual void OnTestEnd(const ::testing::TestInfo &test_info) {
    if (test_info.result()->Failed()) {
      printf("Test %s failed. Printing out all log messages.\n",
             test_info.name());
      fputs("\tThis will include already printed WARNING and up messages.\n",
            stdout);
      TestLogImplementation::GetInstance()->PrintAllMessages();
    }
  }
};

void *DoEnableTestLogging() {
  logging::Init();
  logging::AddImplementation(TestLogImplementation::GetInstance());

  ::testing::UnitTest::GetInstance()->listeners().Append(
      new MyTestEventListener());

  return NULL;
}

Once<void> enable_test_logging_once(DoEnableTestLogging);

}  // namespace

GlobalCoreInstance::GlobalCoreInstance() {
  const size_t kCoreSize = 0x100000;
  global_core = &global_core_data_;
  global_core->owner = 1;
  void *memory = malloc(kCoreSize);
  assert(memory != NULL);
  memset(memory, 0, kCoreSize);

  assert(aos_core_use_address_as_shared_mem(memory, kCoreSize) == 0);

  EnableTestLogging();
}

GlobalCoreInstance::~GlobalCoreInstance() {
  log_uninit();
  free(global_core->mem_struct);
  global_core = NULL;
}

void EnableTestLogging() {
  enable_test_logging_once.Get();
}

}  // namespace testing
}  // namespace common
}  // namespace aos
