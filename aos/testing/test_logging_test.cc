#include "aos/testing/test_logging.h"

#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "aos/logging/logging.h"

namespace aos::testing {

// Tests logging from multiple threads.
// tsan used to complain about this.
TEST(QueueTestutilsTest, MultithreadedLog) {
  EnableTestLogging();

  ::std::thread thread([]() {
    for (int i = 0; i < 1000; ++i) {
      AOS_LOG(INFO, "test from thread\n");
    }
  });
  for (int i = 0; i < 1000; ++i) {
    AOS_LOG(INFO, "not from thread\n");
  }
  thread.join();
}

}  // namespace aos::testing
