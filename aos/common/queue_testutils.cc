#include "aos/common/queue_testutils.h"

#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <assert.h>

#include "gtest/gtest.h"

#include "aos/common/queue.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/once.h"

using ::aos::logging::LogMessage;

namespace aos {
namespace common {
namespace testing {
namespace {

class TestLogImplementation : public logging::HandleMessageLogImplementation {
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

  virtual void HandleMessage(const LogMessage &message) override {
    if (message.level == FATAL) {
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
      fputs("\tIt will also include duplicates of all gtest failures.\n",
            stdout);
      TestLogImplementation::GetInstance()->PrintAllMessages();
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

  return NULL;
}

Once<void> enable_test_logging_once(DoEnableTestLogging);

const size_t kCoreSize = 0x100000;

void TerminateExitHandler() {
  _exit(EXIT_SUCCESS);
}

}  // namespace

GlobalCoreInstance::GlobalCoreInstance() {
  global_core = &global_core_data_;
  global_core->owner = true;
  // Use mmap(2) manually instead of through malloc(3) so that we can pass
  // MAP_SHARED which allows forked processes to communicate using the
  // "shared" memory.
  void *memory = mmap(NULL, kCoreSize, PROT_READ | PROT_WRITE,
                      MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  assert(memory != MAP_FAILED);

  assert(aos_core_use_address_as_shared_mem(memory, kCoreSize) == 0);

  EnableTestLogging();
}

GlobalCoreInstance::~GlobalCoreInstance() {
  assert(munmap(global_core->mem_struct, kCoreSize) == 0);
  global_core = NULL;
}

void EnableTestLogging() {
  enable_test_logging_once.Get();
}

void PreventExit() {
  assert(atexit(TerminateExitHandler) == 0);
}

}  // namespace testing
}  // namespace common
}  // namespace aos
