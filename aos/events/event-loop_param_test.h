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

  // Makes a connected event loop.
  virtual std::unique_ptr<EventLoop> Make() = 0;
  // Makes a primary event loop.  This is the one the tests will try to use for
  // anything blocking.
  virtual std::unique_ptr<EventLoop> MakePrimary() = 0;

  // Runs the loops until they quit.
  virtual void Run() = 0;
};

class AbstractEventLoopTest
    : public ::testing::TestWithParam<std::function<EventLoopTestFactory *()>> {
 public:
  AbstractEventLoopTest() { factory_.reset(GetParam()()); }

  ::std::unique_ptr<EventLoop> Make() { return factory_->Make(); }
  ::std::unique_ptr<EventLoop> MakePrimary() { return factory_->MakePrimary(); }

  void Run() { return factory_->Run(); }
  // You can implement all the usual fixture class members here.
  // To access the test parameter, call GetParam() from class
  // TestWithParam<T>.
 private:
  ::std::unique_ptr<EventLoopTestFactory> factory_;
};

}  // namespace testing
}  // namespace aos

#endif  // _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
