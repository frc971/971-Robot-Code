#include "aos/common/event.h"

#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "aos/common/time.h"
#include "aos/testing/test_logging.h"

namespace aos {
namespace testing {

namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

class EventTest : public ::testing::Test {
 public:
  Event test_event_;

 protected:
  void SetUp() override {
    ::aos::testing::EnableTestLogging();
  }
};

// Makes sure that basic operations with no blocking or anything work.
TEST_F(EventTest, Basic) {
  EXPECT_FALSE(test_event_.Clear());
  EXPECT_FALSE(test_event_.Clear());

  test_event_.Set();
  test_event_.Wait();
  EXPECT_TRUE(test_event_.Clear());
  EXPECT_FALSE(test_event_.Clear());
}

// Tests that tsan understands that events establish a happens-before
// relationship.
TEST_F(EventTest, ThreadSanitizer) {
  for (int i = 0; i < 3000; ++i) {
    int variable = 0;
    test_event_.Clear();
    ::std::thread thread([this, &variable]() {
      test_event_.Wait();
      --variable;
    });
    ++variable;
    test_event_.Set();
    thread.join();
    EXPECT_EQ(0, variable);
  }
}

// Tests that an event blocks correctly.
TEST_F(EventTest, Blocks) {
  monotonic_clock::time_point start_time, finish_time;
  // Without this, it sometimes manages to fail under tsan.
  Event started;
  ::std::thread thread([this, &start_time, &finish_time, &started]() {
    start_time = monotonic_clock::now();
    started.Set();
    test_event_.Wait();
    finish_time = monotonic_clock::now();
  });
  static constexpr auto kWaitTime = chrono::milliseconds(50);
  started.Wait();
  this_thread::sleep_for(kWaitTime);
  test_event_.Set();
  thread.join();
  EXPECT_GE(finish_time - start_time, kWaitTime);
}

TEST_F(EventTest, WaitTimeout) {
  EXPECT_FALSE(test_event_.WaitTimeout(chrono::milliseconds(50)));

  monotonic_clock::time_point start_time, finish_time;
  // Without this, it sometimes manages to fail under tsan.
  Event started;
  ::std::thread thread([this, &start_time, &finish_time, &started]() {
    start_time = monotonic_clock::now();
    started.Set();
    EXPECT_TRUE(test_event_.WaitTimeout(chrono::milliseconds(500)));
    finish_time = monotonic_clock::now();
  });
  constexpr auto kWaitTime = chrono::milliseconds(50);
  started.Wait();
  this_thread::sleep_for(kWaitTime);
  test_event_.Set();
  thread.join();
  EXPECT_GE(finish_time - start_time, kWaitTime);
}

}  // namespace testing
}  // namespace aos
