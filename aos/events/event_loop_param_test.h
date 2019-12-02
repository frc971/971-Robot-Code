#ifndef _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
#define _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_

#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

class EventLoopTestFactory {
 public:
  EventLoopTestFactory()
      : flatbuffer_(JsonToFlatbuffer("{\n"
                                     "  \"channels\": [ \n"
                                     "    {\n"
                                     "      \"name\": \"/aos\",\n"
                                     "      \"type\": \"aos.timing.Report\"\n"
                                     "    },\n"
                                     "    {\n"
                                     "      \"name\": \"/test\",\n"
                                     "      \"type\": \"aos.TestMessage\"\n"
                                     "    },\n"
                                     "    {\n"
                                     "      \"name\": \"/test1\",\n"
                                     "      \"type\": \"aos.TestMessage\"\n"
                                     "    },\n"
                                     "    {\n"
                                     "      \"name\": \"/test2\",\n"
                                     "      \"type\": \"aos.TestMessage\"\n"
                                     "    }\n"
                                     "  ]\n"
                                     "}\n",
                                     Configuration::MiniReflectTypeTable())) {}

  virtual ~EventLoopTestFactory() {}

  // Makes a connected event loop.
  virtual std::unique_ptr<EventLoop> Make(std::string_view name) = 0;
  // Makes a primary event loop.  This is the one the tests will try to use for
  // anything blocking.
  virtual std::unique_ptr<EventLoop> MakePrimary(std::string_view name) = 0;

  // Runs the loops until they quit.
  virtual void Run() = 0;

  // Quits the loops.
  virtual void Exit() = 0;

  // Advances time by sleeping.  Can't be called from inside a loop.
  virtual void SleepFor(::std::chrono::nanoseconds duration) = 0;

  const Configuration *configuration() { return &flatbuffer_.message(); }

 private:
  FlatbufferDetachedBuffer<Configuration> flatbuffer_;
};

class AbstractEventLoopTestBase
    : public ::testing::TestWithParam<std::function<EventLoopTestFactory *()>> {
 public:
  AbstractEventLoopTestBase() { factory_.reset(GetParam()()); }

  ::std::unique_ptr<EventLoop> Make(std::string_view name = "") {
    std::string name_copy(name);
    if (name == "") {
      name_copy = "loop";
      name_copy += std::to_string(event_loop_count_);
    }
    ++event_loop_count_;
    return factory_->Make(name_copy);
  }
  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name = "primary") {
    ++event_loop_count_;
    return factory_->MakePrimary(name);
  }

  void Run() { return factory_->Run(); }

  void Exit() { return factory_->Exit(); }

  void SleepFor(::std::chrono::nanoseconds duration) {
    return factory_->SleepFor(duration);
  }

  // Ends the given event loop at the given time from now.
  void EndEventLoop(EventLoop *loop, ::std::chrono::milliseconds duration) {
    auto end_timer = loop->AddTimer([this]() { this->Exit(); });
    end_timer->Setup(loop->monotonic_now() +
                     ::std::chrono::milliseconds(duration));
    end_timer->set_name("end");
  }

  // You can implement all the usual fixture class members here.
  // To access the test parameter, call GetParam() from class
  // TestWithParam<T>.
 private:
  ::std::unique_ptr<EventLoopTestFactory> factory_;

  int event_loop_count_ = 0;
};

typedef AbstractEventLoopTestBase AbstractEventLoopDeathTest;
typedef AbstractEventLoopTestBase AbstractEventLoopTest;

}  // namespace testing
}  // namespace aos

#endif  // _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
