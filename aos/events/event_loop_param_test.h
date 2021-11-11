#ifndef _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_
#define _AOS_EVENTS_EVENT_LOOP_PARAM_TEST_H_

#include <initializer_list>
#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/test_message_generated.h"
#include "aos/events/test_message_schema.h"
#include "aos/events/timing_report_schema.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/logging/log_message_schema.h"
#include "aos/network/message_bridge_client_schema.h"
#include "aos/network/message_bridge_server_schema.h"
#include "aos/network/timestamp_schema.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

class EventLoopTestFactory {
 public:
  EventLoopTestFactory()
      : flatbuffer_(configuration::AddSchema(
            R"config({
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
      "type": "aos.TestMessage",
      "frequency": 800
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
})config",
            {aos::FlatbufferSpan<reflection::Schema>(
                 logging::LogMessageFbsSchema()),
             aos::FlatbufferSpan<reflection::Schema>(timing::ReportSchema()),
             aos::FlatbufferSpan<reflection::Schema>(TestMessageSchema())})) {}

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

  // Sets the config to a config with a max size with an invalid alignment.
  void InvalidChannelAlignment() {
    flatbuffer_ = configuration::AddSchema(
        R"config({
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
      "type": "aos.TestMessage",
      "max_size": 13
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
})config",
        {aos::FlatbufferSpan<reflection::Schema>(
             logging::LogMessageFbsSchema()),
         aos::FlatbufferSpan<reflection::Schema>(timing::ReportSchema()),
         aos::FlatbufferSpan<reflection::Schema>(TestMessageSchema())});
  }

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
      "num_readers": 10,
      "frequency": 800
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

    flatbuffer_ = configuration::AddSchema(
        kJson, {aos::FlatbufferSpan<reflection::Schema>(
                    logging::LogMessageFbsSchema()),
                aos::FlatbufferSpan<reflection::Schema>(timing::ReportSchema()),
                aos::FlatbufferSpan<reflection::Schema>(TestMessageSchema())});
  }

  void EnableNodes(std::string_view my_node) {
    static const std::string kJson = R"config({
  "channels": [
    {
      "name": "/me/aos",
      "type": "aos.logging.LogMessageFbs",
      "source_node": "me"
    },
    {
      "name": "/them/aos",
      "type": "aos.logging.LogMessageFbs",
      "source_node": "them"
    },
    {
      "name": "/me/aos",
      "type": "aos.message_bridge.Timestamp",
      "source_node": "me",
      "destination_nodes": [
        {
          "name": "them"
        }
      ]
    },
    {
      "name": "/them/aos",
      "type": "aos.message_bridge.Timestamp",
      "source_node": "them",
      "destination_nodes": [
        {
          "name": "me"
        }
      ]
    },
    {
      "name": "/me/aos",
      "type": "aos.message_bridge.ServerStatistics",
      "source_node": "me",
      "frequency": 2
    },
    {
      "name": "/them/aos",
      "type": "aos.message_bridge.ServerStatistics",
      "source_node": "them",
      "frequency": 2
    },
    {
      "name": "/me/aos",
      "type": "aos.message_bridge.ClientStatistics",
      "source_node": "me",
      "frequency": 2
    },
    {
      "name": "/them/aos",
      "type": "aos.message_bridge.ClientStatistics",
      "source_node": "them",
      "frequency": 2
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
    },
    {
      "name": "/test_forward",
      "type": "aos.TestMessage",
      "source_node": "them",
      "destination_nodes": [
        {
          "name": "me"
        }
      ]
    },
    {
      "name": "/test_noforward",
      "type": "aos.TestMessage",
      "source_node": "them"
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
        "name": "/aos*",
        "source_node": "me"
      },
      "rename": {
        "name": "/me/aos"
      }
    },
    {
      "match": {
        "name": "/aos*",
        "source_node": "them"
      },
      "rename": {
        "name": "/them/aos"
      }
    }
  ]
})config";

    flatbuffer_ = configuration::MergeConfiguration(
        configuration::MergeConfiguration(
            aos::FlatbufferDetachedBuffer<Configuration>(
                JsonToFlatbuffer<Configuration>(kJson))),
        {aos::FlatbufferSpan<reflection::Schema>(
             logging::LogMessageFbsSchema()),
         aos::FlatbufferSpan<reflection::Schema>(timing::ReportSchema()),
         aos::FlatbufferSpan<reflection::Schema>(TestMessageSchema()),
         aos::FlatbufferSpan<reflection::Schema>(
             message_bridge::ClientStatisticsSchema()),
         aos::FlatbufferSpan<reflection::Schema>(
             message_bridge::ServerStatisticsSchema()),
         aos::FlatbufferSpan<reflection::Schema>(
             message_bridge::TimestampSchema())});

    my_node_ = configuration::GetNode(&flatbuffer_.message(), my_node);
  }

  const Node *my_node() const { return my_node_; }

  const Configuration *configuration() { return &flatbuffer_.message(); }

 private:
  FlatbufferDetachedBuffer<Configuration> flatbuffer_;

  const Node *my_node_ = nullptr;
};

enum class DoTimingReports { kYes, kNo };

class AbstractEventLoopTest
    : public ::testing::TestWithParam<
          std::tuple<std::function<EventLoopTestFactory *()>, ReadMethod,
                     DoTimingReports>> {
 public:
  AbstractEventLoopTest() : factory_(std::get<0>(GetParam())()) {
    if (read_method() == ReadMethod::PIN) {
      factory_->PinReads();
    }
  }

  ReadMethod read_method() const { return std::get<1>(GetParam()); }
  DoTimingReports do_timing_reports() const { return std::get<2>(GetParam()); }

  ::std::unique_ptr<EventLoop> Make(std::string_view name = "");

  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name = "primary") {
    ++event_loop_count_;
    auto result = factory_->MakePrimary(name);
    if (do_timing_reports() == DoTimingReports::kNo) {
      result->SkipTimingReport();
    }
    return result;
  }

  void InvalidChannelAlignment() { factory_->InvalidChannelAlignment(); }

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
