#ifndef _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
#define _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_

#include <vector>

#include "aos/events/event-loop.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

class EventLoopTestFactory {
 public:
  virtual ~EventLoopTestFactory() {}

  virtual std::unique_ptr<EventLoop> Make() = 0;
};

class AbstractEventLoopTest
    : public ::testing::TestWithParam<std::function<EventLoopTestFactory *()>> {
 public:
  AbstractEventLoopTest() { factory_.reset(GetParam()()); }

  std::unique_ptr<EventLoop> Make() { return factory_->Make(); }
  // You can implement all the usual fixture class members here.
  // To access the test parameter, call GetParam() from class
  // TestWithParam<T>.
 private:
  std::unique_ptr<EventLoopTestFactory> factory_;
};

}  // namespace testing
}  // namespace aos

#endif  // _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
