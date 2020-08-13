#ifndef _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
#define _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_

#include <initializer_list>
#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/test_message_generated.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

class EventLoopTestFactory {
 public:
  EventLoopTestFactory()
      : flatbuffer_(JsonToFlatbuffer<Configuration>(R"config({
  "channels": [
    {
      "name": "/aos",
      "type": "aos.logging.LogMessageFbs"
    },
    {
      "name": "/aos",
      "type": "aos.timing.Report"
    },
    {
      "name": "/test",
      "type": "aos.TestMessage"
    },
    {
      "name": "/test1",
      "type": "aos.TestMessage"
    },
    {
      "name": "/test2",
      "type": "aos.TestMessage"
    }
  ]
})config")) {}

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

  void PinReads() {
    static const std::string kJson = R"config({
  "channels": [
    {
      "name": "/aos",
      "type": "aos.logging.LogMessageFbs",
      "read_method": "PIN",
      "num_readers": 10
    },
    {
      "name": "/aos",
      "type": "aos.timing.Report",
      "read_method": "PIN",
      "num_readers": 10
    },
    {
      "name": "/test",
      "type": "aos.TestMessage",
      "read_method": "PIN",
      "num_readers": 10
    },
    {
      "name": "/test1",
      "type": "aos.TestMessage",
      "read_method": "PIN",
      "num_readers": 10
    },
    {
      "name": "/test2",
      "type": "aos.TestMessage",
      "read_method": "PIN",
      "num_readers": 10
    }
  ]
})config";

    flatbuffer_ = FlatbufferDetachedBuffer<Configuration>(
        JsonToFlatbuffer(kJson, Configuration::MiniReflectTypeTable()));
  }

  void EnableNodes(std::string_view my_node) {
    static const std::string kJson = R"config({
  "channels": [
    {
      "name": "/aos/me",
      "type": "aos.logging.LogMessageFbs",
      "source_node": "me"
    },
    {
      "name": "/aos/them",
      "type": "aos.logging.LogMessageFbs",
      "source_node": "them"
    },
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
      "name": "me",
      "hostname": "myhostname"
    },
    {
      "name": "them",
      "hostname": "themhostname"
    }
  ],
  "maps": [
    {
      "match": {
        "name": "/aos",
        "type": "aos.logging.LogMessageFbs",
        "source_node": "me"
      },
      "rename": {
        "name": "/aos/me"
      }
    },
    {
      "match": {
        "name": "/aos",
        "type": "aos.logging.LogMessageFbs",
        "source_node": "them"
      },
      "rename": {
        "name": "/aos/them"
      }
    }
  ]
})config";

    flatbuffer_ = FlatbufferDetachedBuffer<Configuration>(
        JsonToFlatbuffer(kJson, Configuration::MiniReflectTypeTable()));

    my_node_ = configuration::GetNode(&flatbuffer_.message(), my_node);
  }

  const Node *my_node() const { return my_node_; }

  const Configuration *configuration() { return &flatbuffer_.message(); }

 private:
  FlatbufferDetachedBuffer<Configuration> flatbuffer_;

  const Node *my_node_ = nullptr;
};

class AbstractEventLoopTest
    : public ::testing::TestWithParam<
          std::tuple<std::function<EventLoopTestFactory *()>, ReadMethod>> {
 public:
  AbstractEventLoopTest() : factory_(std::get<0>(GetParam())()) {
    if (read_method() == ReadMethod::PIN) {
      factory_->PinReads();
    }
  }

  ReadMethod read_method() const { return std::get<1>(GetParam()); }

  ::std::unique_ptr<EventLoop> Make(std::string_view name = "");

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

  const Node *my_node() const { return factory_->my_node(); }

  // Ends the given event loop at the given time from now.
  void EndEventLoop(EventLoop *loop, ::std::chrono::milliseconds duration) {
    auto end_timer = loop->AddTimer([this]() { this->Exit(); });
    end_timer->Setup(loop->monotonic_now() + duration);
    end_timer->set_name("end");
  }

  // Verifies that the buffer_index values for all of the given objects are
  // consistent.
  void VerifyBuffers(
      int number_buffers,
      std::vector<std::reference_wrapper<const Fetcher<TestMessage>>> fetchers,
      std::vector<std::reference_wrapper<const Sender<TestMessage>>> senders);

 private:
  const ::std::unique_ptr<EventLoopTestFactory> factory_;

  int event_loop_count_ = 0;
};

using AbstractEventLoopDeathTest = AbstractEventLoopTest;

}  // namespace testing
}  // namespace aos

#endif  // _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
