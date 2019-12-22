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

  void EnableNodes(std::string_view my_node) {
    std::string json = std::string(R"config({
  "channels": [
    {
      "name": "/aos",
      "type": "aos.timing.Report",
      "source_node": "me"
    },
    {
      "name": "/test",
      "type": "aos.TestMessage",
      "source_node": "me"
    },
    {
      "name": "/test1",
      "type": "aos.TestMessage",
      "source_node": "me"
    },
    {
      "name": "/test2",
      "type": "aos.TestMessage",
      "source_node": "me"
    }
  ],
  "nodes": [
    {
      "name": ")config") +
                       std::string(my_node) + R"config(",
      "hostname": "myhostname"
    }
  ]
})config";

    flatbuffer_ = FlatbufferDetachedBuffer<Configuration>(
        JsonToFlatbuffer(json, Configuration::MiniReflectTypeTable()));

    my_node_ = my_node;
  }

  std::string_view my_node() const { return my_node_; }

  const Configuration *configuration() { return &flatbuffer_.message(); }

 private:
  FlatbufferDetachedBuffer<Configuration> flatbuffer_;

  std::string my_node_;
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

  void EnableNodes(std::string_view my_node) { factory_->EnableNodes(my_node); }

  void Run() { return factory_->Run(); }

  void Exit() { return factory_->Exit(); }

  void SleepFor(::std::chrono::nanoseconds duration) {
    return factory_->SleepFor(duration);
  }

  const Configuration *configuration() { return factory_->configuration(); }

  std::string_view my_node() const { return factory_->my_node(); }

  // Ends the given event loop at the given time from now.
  void EndEventLoop(EventLoop *loop, ::std::chrono::milliseconds duration) {
    auto end_timer = loop->AddTimer([this]() { this->Exit(); });
    end_timer->Setup(loop->monotonic_now() + duration);
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
