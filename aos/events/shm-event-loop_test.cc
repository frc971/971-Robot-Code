#include "aos/events/shm-event-loop.h"

#include "aos/events/event-loop_param_test.h"
#include "aos/testing/test_shm.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {
namespace {

class ShmEventLoopTestFactory : public EventLoopTestFactory {
 public:
  std::unique_ptr<EventLoop> Make() override {
    return std::unique_ptr<EventLoop>(new ShmEventLoop());
  }

  ::aos::testing::TestSharedMemory my_shm_;
};

INSTANTIATE_TEST_CASE_P(ShmEventLoopTest, AbstractEventLoopTest,
                        ::testing::Values([]() {
                          return new ShmEventLoopTestFactory();
                        }));

}  // namespace
}  // namespace testing
}  // namespace aos
