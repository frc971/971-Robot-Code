#include "aos/common/event.h"

#include <thread>

#include "gtest/gtest.h"

#include "aos/common/queue_testutils.h"
#include "aos/common/time.h"

namespace aos {
namespace testing {

class EventTest : public ::testing::Test {
 public:
  Event test_event;

 protected:
  void SetUp() override {
    ::aos::common::testing::EnableTestLogging();
  }
};

// Makes sure that basic operations with no blocking or anything work.
TEST_F(EventTest, Basic) {
  EXPECT_FALSE(test_event.Clear());
  EXPECT_FALSE(test_event.Clear());

  test_event.Set();
  test_event.Wait();
  EXPECT_TRUE(test_event.Clear());
  EXPECT_FALSE(test_event.Clear());
}

// Tests that tsan understands that events establish a happens-before
// relationship.
TEST_F(EventTest, ThreadSanitizer) {
  for (int i = 0; i < 3000; ++i) {
    int variable = 0;
    test_event.Clear();
    ::std::thread thread([this, &variable]() {
      test_event.Wait();
      --variable;
    });
    ++variable;
    test_event.Set();
    thread.join();
    EXPECT_EQ(0, variable);
  }
}

// Tests that an event blocks correctly.
TEST_F(EventTest, Blocks) {
  time::Time start_time, finish_time;
  // Without this, it sometimes manages to fail under tsan.
  Event started;
  ::std::thread thread([this, &start_time, &finish_time, &started]() {
    start_time = time::Time::Now();
    started.Set();
    test_event.Wait();
    finish_time = time::Time::Now();
  });
  static const time::Time kWaitTime = time::Time::InSeconds(0.05);
  started.Wait();
  time::SleepFor(kWaitTime);
  test_event.Set();
  thread.join();
  EXPECT_GE(finish_time - start_time, kWaitTime);
}

}  // namespace testing
}  // namespace aos
