#include "aos/events/event_loop_param_test.h"

#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include "aos/flatbuffer_merge.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {
namespace {
namespace chrono = ::std::chrono;
}  // namespace

::std::unique_ptr<EventLoop> AbstractEventLoopTest::Make(
    std::string_view name) {
  std::string name_copy(name);
  if (name == "") {
    name_copy = "loop";
    name_copy += std::to_string(event_loop_count_);
  }
  ++event_loop_count_;
  return factory_->Make(name_copy);
}

void AbstractEventLoopTest::VerifyBuffers(
    int number_buffers,
    std::vector<std::reference_wrapper<const Fetcher<TestMessage>>> fetchers,
    std::vector<std::reference_wrapper<const Sender<TestMessage>>> senders) {
  // The buffers which are in a sender.
  std::unordered_set<int> in_sender;
  for (const Sender<TestMessage> &sender : senders) {
    const int this_buffer = sender.buffer_index();
    CHECK_GE(this_buffer, 0);
    CHECK_LT(this_buffer, number_buffers);
    CHECK(in_sender.insert(this_buffer).second) << ": " << this_buffer;
  }

  if (read_method() != ReadMethod::PIN) {
    // If we're not using PIN, we can't really verify anything about what
    // buffers the fetchers have.
    return;
  }

  // Mapping from TestMessage::value to buffer index.
  std::unordered_map<int, int> fetcher_values;
  for (const Fetcher<TestMessage> &fetcher : fetchers) {
    if (!fetcher.get()) {
      continue;
    }
    const int this_buffer = fetcher.context().buffer_index;
    CHECK_GE(this_buffer, 0);
    CHECK_LT(this_buffer, number_buffers);
    CHECK(in_sender.count(this_buffer) == 0) << ": " << this_buffer;
    const auto insert_result = fetcher_values.insert(
        std::make_pair(fetcher.get()->value(), this_buffer));
    if (!insert_result.second) {
      CHECK_EQ(this_buffer, insert_result.first->second);
    }
  }
}

// Tests that watcher can receive messages from a sender.
// Also tests that OnRun() works.
TEST_P(AbstractEventLoopTest, Basic) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");

  bool happened = false;

  loop2->OnRun([&]() {
    happened = true;

    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  loop2->MakeWatcher("/test", [&](const TestMessage &message) {
    EXPECT_EQ(message.value(), 200);
    this->Exit();
  });

  EXPECT_FALSE(happened);
  Run();
  EXPECT_TRUE(happened);
}

// Tests that watcher can receive messages from a sender, sent via SendDetached.
TEST_P(AbstractEventLoopTest, BasicSendDetached) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");

  FlatbufferDetachedBuffer<TestMessage> detached =
      flatbuffers::DetachedBuffer();
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(100);
    detached = msg.Detach(builder.Finish());
  }
  detached = flatbuffers::DetachedBuffer();
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    detached = msg.Detach(builder.Finish());
  }
  ASSERT_TRUE(sender.SendDetached(std::move(detached)));

  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");
  ASSERT_TRUE(fetcher.Fetch());
  EXPECT_EQ(fetcher->value(), 200);
}

// Verifies that a no-arg watcher will not have a data pointer.
TEST_P(AbstractEventLoopTest, NoArgNoData) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");

  bool happened = false;

  loop2->OnRun([&]() {
    happened = true;

    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  loop2->MakeNoArgWatcher<TestMessage>("/test", [&]() {
    EXPECT_GT(loop2->context().size, 0u);
    EXPECT_EQ(nullptr, loop2->context().data);
    EXPECT_EQ(-1, loop2->context().buffer_index);
    this->Exit();
  });

  EXPECT_FALSE(happened);
  Run();
  EXPECT_TRUE(happened);
}

// Tests that no-arg watcher can receive messages from a sender.
// Also tests that OnRun() works.
TEST_P(AbstractEventLoopTest, BasicNoArg) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");

  bool happened = false;

  loop2->OnRun([&]() {
    happened = true;

    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  aos::Fetcher<TestMessage> fetcher = loop2->MakeFetcher<TestMessage>("/test");
  loop2->MakeNoArgWatcher<TestMessage>("/test", [&]() {
    ASSERT_TRUE(fetcher.Fetch());
    EXPECT_EQ(fetcher->value(), 200);
    this->Exit();
  });

  EXPECT_FALSE(happened);
  Run();
  EXPECT_TRUE(happened);
}

// Tests that a watcher can be created with an std::function.
TEST_P(AbstractEventLoopTest, BasicFunction) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");

  bool happened = false;

  loop2->OnRun([&]() {
    happened = true;

    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  loop2->MakeWatcher("/test", std::function<void(const TestMessage &)>(
                                  [&](const TestMessage &message) {
                                    EXPECT_EQ(message.value(), 200);
                                    this->Exit();
                                  }));

  EXPECT_FALSE(happened);
  Run();
  EXPECT_TRUE(happened);
}

// Tests that watcher can receive messages from two senders.
// Also tests that OnRun() works.
TEST_P(AbstractEventLoopTest, BasicTwoSenders) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  aos::Sender<TestMessage> sender1 = loop1->MakeSender<TestMessage>("/test");
  aos::Sender<TestMessage> sender2 = loop1->MakeSender<TestMessage>("/test");

  bool happened = false;

  loop2->OnRun([&]() {
    happened = true;

    {
      aos::Sender<TestMessage>::Builder msg = sender1.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
    {
      aos::Sender<TestMessage>::Builder msg = sender2.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
  });

  int messages_received = 0;
  loop2->MakeWatcher("/test", [&](const TestMessage &message) {
    EXPECT_EQ(message.value(), 200);
    this->Exit();
    ++messages_received;
  });

  EXPECT_FALSE(happened);
  Run();
  EXPECT_TRUE(happened);
  EXPECT_EQ(messages_received, 2);
}

// Tests that a fetcher can fetch from a sender.
// Also tests that OnRun() works.
TEST_P(AbstractEventLoopTest, FetchWithoutRun) {
  auto loop1 = Make();
  auto loop2 = Make();
  auto loop3 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  EXPECT_FALSE(fetcher.Fetch());
  EXPECT_EQ(fetcher.get(), nullptr);

  EXPECT_EQ(fetcher.context().monotonic_event_time, monotonic_clock::min_time);
  EXPECT_EQ(fetcher.context().monotonic_remote_time, monotonic_clock::min_time);
  EXPECT_EQ(fetcher.context().realtime_event_time, realtime_clock::min_time);
  EXPECT_EQ(fetcher.context().realtime_remote_time, realtime_clock::min_time);
  EXPECT_EQ(fetcher.context().queue_index, 0xffffffffu);
  EXPECT_EQ(fetcher.context().size, 0u);
  EXPECT_EQ(fetcher.context().data, nullptr);
  EXPECT_EQ(fetcher.context().buffer_index, -1);

  aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
  TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
  builder.add_value(200);
  ASSERT_TRUE(msg.Send(builder.Finish()));

  EXPECT_TRUE(fetcher.Fetch());
  ASSERT_FALSE(fetcher.get() == nullptr);
  EXPECT_EQ(fetcher.get()->value(), 200);

  const chrono::milliseconds kEpsilon(100);

  const aos::monotonic_clock::time_point monotonic_now = loop2->monotonic_now();
  const aos::realtime_clock::time_point realtime_now = loop2->realtime_now();
  EXPECT_EQ(fetcher.context().monotonic_event_time,
            fetcher.context().monotonic_remote_time);
  EXPECT_EQ(fetcher.context().realtime_event_time,
            fetcher.context().realtime_remote_time);

  EXPECT_GE(fetcher.context().monotonic_event_time, monotonic_now - kEpsilon);
  EXPECT_LE(fetcher.context().monotonic_event_time, monotonic_now + kEpsilon);
  EXPECT_GE(fetcher.context().realtime_event_time, realtime_now - kEpsilon);
  EXPECT_LE(fetcher.context().realtime_event_time, realtime_now + kEpsilon);
  EXPECT_EQ(fetcher.context().queue_index, 0x0u);
  EXPECT_EQ(fetcher.context().size, 20u);
  EXPECT_NE(fetcher.context().data, nullptr);
  if (read_method() == ReadMethod::PIN) {
    EXPECT_GE(fetcher.context().buffer_index, 0);
    EXPECT_LT(fetcher.context().buffer_index,
              loop2->NumberBuffers(fetcher.channel()));
  } else {
    EXPECT_EQ(fetcher.context().buffer_index, -1);
  }
}

// Tests that watcher will receive all messages sent if they are sent after
// initialization and before running.
TEST_P(AbstractEventLoopTest, DoubleSendAtStartup) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  ::std::vector<int> values;

  loop2->MakeWatcher("/test", [&](const TestMessage &message) {
    values.push_back(message.value());
    if (values.size() == 2) {
      this->Exit();
    }
  });

  // Before Run, should be ignored.
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(199);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  loop2->OnRun([&]() {
    {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
    {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(201);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
  });

  Run();

  EXPECT_THAT(values, ::testing::ElementsAreArray({200, 201}));
}

// Tests that watcher will not receive messages sent before the watcher is
// created.
TEST_P(AbstractEventLoopTest, DoubleSendAfterStartup) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  ::std::vector<int> values;

  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(201);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  loop2->MakeWatcher("/test", [&](const TestMessage &message) {
    values.push_back(message.value());
  });

  // Add a timer to actually quit.
  auto test_timer = loop2->AddTimer([this]() { this->Exit(); });
  loop2->OnRun([&test_timer, &loop2]() {
    test_timer->Setup(loop2->monotonic_now(), ::std::chrono::milliseconds(100));
  });

  Run();
  EXPECT_EQ(0, values.size());
}

// Tests that FetchNext gets all the messages sent after it is constructed.
TEST_P(AbstractEventLoopTest, FetchNext) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");
  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  ::std::vector<int> values;

  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(201);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  // Add a timer to actually quit.
  auto test_timer = loop2->AddTimer([&fetcher, &values, this]() {
    while (fetcher.FetchNext()) {
      values.push_back(fetcher.get()->value());
    }
    this->Exit();
  });

  loop2->OnRun([&test_timer, &loop2]() {
    test_timer->Setup(loop2->monotonic_now(), ::std::chrono::milliseconds(100));
  });

  Run();
  EXPECT_THAT(values, ::testing::ElementsAreArray({200, 201}));
}

// Tests that FetchNext gets no messages sent before it is constructed.
TEST_P(AbstractEventLoopTest, FetchNextAfterSend) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  ::std::vector<int> values;

  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(201);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  // Add a timer to actually quit.
  auto test_timer = loop2->AddTimer([&fetcher, &values, this]() {
    while (fetcher.FetchNext()) {
      values.push_back(fetcher.get()->value());
    }
    this->Exit();
  });

  loop2->OnRun([&test_timer, &loop2]() {
    test_timer->Setup(loop2->monotonic_now(), ::std::chrono::milliseconds(100));
  });

  Run();
  EXPECT_THAT(0, values.size());
}

// Tests that Fetch returns the last message created before the loop was
// started.
TEST_P(AbstractEventLoopTest, FetchDataFromBeforeCreation) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  ::std::vector<int> values;

  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(201);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  // Add a timer to actually quit.
  auto test_timer = loop2->AddTimer([&fetcher, &values, this]() {
    if (fetcher.Fetch()) {
      values.push_back(fetcher.get()->value());
    }
    // Do it again to make sure we don't double fetch.
    if (fetcher.Fetch()) {
      values.push_back(fetcher.get()->value());
    }
    this->Exit();
  });

  loop2->OnRun([&test_timer, &loop2]() {
    test_timer->Setup(loop2->monotonic_now(), ::std::chrono::milliseconds(100));
  });

  Run();
  EXPECT_THAT(values, ::testing::ElementsAreArray({201}));
}

// Tests that Fetch and FetchNext interleave as expected.
TEST_P(AbstractEventLoopTest, FetchAndFetchNextTogether) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  ::std::vector<int> values;

  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(201);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  // Add a timer to actually quit.
  auto test_timer = loop2->AddTimer([&fetcher, &values, &sender, this]() {
    if (fetcher.Fetch()) {
      values.push_back(fetcher.get()->value());
    }

    {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(202);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
    {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(203);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
    {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(204);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }

    if (fetcher.FetchNext()) {
      values.push_back(fetcher.get()->value());
    }

    if (fetcher.Fetch()) {
      values.push_back(fetcher.get()->value());
    }

    this->Exit();
  });

  loop2->OnRun([&test_timer, &loop2]() {
    test_timer->Setup(loop2->monotonic_now(), ::std::chrono::milliseconds(100));
  });

  Run();
  EXPECT_THAT(values, ::testing::ElementsAreArray({201, 202, 204}));
}

// Tests that FetchNext behaves correctly when we get two messages in the queue
// but don't consume the first until after the second has been sent.
TEST_P(AbstractEventLoopTest, FetchNextTest) {
  auto send_loop = Make();
  auto fetch_loop = Make();
  auto sender = send_loop->MakeSender<TestMessage>("/test");
  Fetcher<TestMessage> fetcher = fetch_loop->MakeFetcher<TestMessage>("/test");

  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(100);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }

  ASSERT_TRUE(fetcher.FetchNext());
  ASSERT_NE(nullptr, fetcher.get());
  EXPECT_EQ(100, fetcher.get()->value());

  ASSERT_TRUE(fetcher.FetchNext());
  ASSERT_NE(nullptr, fetcher.get());
  EXPECT_EQ(200, fetcher.get()->value());

  // When we run off the end of the queue, expect to still have the old message:
  ASSERT_FALSE(fetcher.FetchNext());
  ASSERT_NE(nullptr, fetcher.get());
  EXPECT_EQ(200, fetcher.get()->value());
}

// Verify that a fetcher still holds its data, even after falling behind.
TEST_P(AbstractEventLoopTest, FetcherBehindData) {
  auto send_loop = Make();
  auto fetch_loop = Make();
  auto sender = send_loop->MakeSender<TestMessage>("/test");
  Fetcher<TestMessage> fetcher = fetch_loop->MakeFetcher<TestMessage>("/test");
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(1);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }
  ASSERT_TRUE(fetcher.Fetch());
  EXPECT_EQ(1, fetcher.get()->value());
  for (int i = 0; i < 300; ++i) {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(i + 2);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  }
  EXPECT_EQ(1, fetcher.get()->value());
}

// Try a bunch of orderings of operations with fetchers and senders. Verify that
// all the fetchers have the correct data at each step.
TEST_P(AbstractEventLoopTest, FetcherPermutations) {
  for (int max_save = 0; max_save < 5; ++max_save) {
    SCOPED_TRACE("max_save=" + std::to_string(max_save));

    auto send_loop = Make();
    auto fetch_loop = Make();
    auto sender = send_loop->MakeSender<TestMessage>("/test");
    const auto send_message = [&sender](int i) {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(i);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    };
    std::vector<Fetcher<TestMessage>> fetchers;
    for (int i = 0; i < 10; ++i) {
      fetchers.emplace_back(fetch_loop->MakeFetcher<TestMessage>("/test"));
    }
    send_message(1);
    const auto verify_buffers = [&]() {
      std::vector<std::reference_wrapper<const Fetcher<TestMessage>>>
          fetchers_copy;
      for (const auto &fetcher : fetchers) {
        fetchers_copy.emplace_back(fetcher);
      }
      std::vector<std::reference_wrapper<const Sender<TestMessage>>>
          senders_copy;
      senders_copy.emplace_back(sender);
      VerifyBuffers(send_loop->NumberBuffers(sender.channel()), fetchers_copy,
                    senders_copy);
    };
    for (auto &fetcher : fetchers) {
      ASSERT_TRUE(fetcher.Fetch());
      verify_buffers();
      EXPECT_EQ(1, fetcher.get()->value());
    }

    for (int save = 1; save <= max_save; ++save) {
      SCOPED_TRACE("save=" + std::to_string(save));
      send_message(100 + save);
      verify_buffers();
      for (size_t i = 0; i < fetchers.size() - save; ++i) {
        SCOPED_TRACE("fetcher=" + std::to_string(i));
        ASSERT_TRUE(fetchers[i].Fetch());
        verify_buffers();
        EXPECT_EQ(100 + save, fetchers[i].get()->value());
      }
      for (size_t i = fetchers.size() - save; i < fetchers.size() - 1; ++i) {
        SCOPED_TRACE("fetcher=" + std::to_string(i));
        EXPECT_EQ(100 + (fetchers.size() - 1 - i), fetchers[i].get()->value());
      }
      EXPECT_EQ(1, fetchers.back().get()->value());
    }

    for (int i = 0; i < 300; ++i) {
      send_message(200 + i);
      verify_buffers();
    }

    for (size_t i = 0; i < fetchers.size() - max_save; ++i) {
      SCOPED_TRACE("fetcher=" + std::to_string(i));
      if (max_save > 0) {
        EXPECT_EQ(100 + max_save, fetchers[i].get()->value());
      } else {
        EXPECT_EQ(1, fetchers[i].get()->value());
      }
    }
    for (size_t i = fetchers.size() - max_save; i < fetchers.size() - 1; ++i) {
      SCOPED_TRACE("fetcher=" + std::to_string(i));
      EXPECT_EQ(100 + (fetchers.size() - 1 - i), fetchers[i].get()->value());
    }
    EXPECT_EQ(1, fetchers.back().get()->value());
  }
}

// Verify that making a fetcher and watcher for "/test" succeeds.
TEST_P(AbstractEventLoopTest, FetcherAndWatcher) {
  auto loop = Make();
  auto fetcher = loop->MakeFetcher<TestMessage>("/test");
  loop->MakeWatcher("/test", [&](const TestMessage &) {});
}

// Verify that making 2 fetchers for "/test" succeeds.
TEST_P(AbstractEventLoopTest, TwoFetcher) {
  auto loop = Make();
  auto fetcher = loop->MakeFetcher<TestMessage>("/test");
  auto fetcher2 = loop->MakeFetcher<TestMessage>("/test");
}

// Verify that registering a watcher for an invalid channel name dies.
TEST_P(AbstractEventLoopDeathTest, InvalidChannelName) {
  auto loop = Make();
  EXPECT_DEATH(
      { loop->MakeWatcher("/test/invalid", [&](const TestMessage &) {}); },
      "/test/invalid");
  EXPECT_DEATH(
      { loop->MakeNoArgWatcher<TestMessage>("/test/invalid", [&]() {}); },
      "/test/invalid");
}

// Verify that registering a watcher twice for "/test" fails.
TEST_P(AbstractEventLoopDeathTest, TwoWatcher) {
  auto loop = Make();
  loop->MakeWatcher("/test", [&](const TestMessage &) {});
  EXPECT_DEATH(loop->MakeWatcher("/test", [&](const TestMessage &) {}),
               "/test");
  EXPECT_DEATH(loop->MakeNoArgWatcher<TestMessage>("/test", [&]() {}), "/test");
}

// Verify that registering a no-arg watcher twice for "/test" fails.
TEST_P(AbstractEventLoopDeathTest, TwoNoArgWatcher) {
  auto loop = Make();
  loop->MakeNoArgWatcher<TestMessage>("/test", [&]() {});
  EXPECT_DEATH(loop->MakeWatcher("/test", [&](const TestMessage &) {}),
               "/test");
  EXPECT_DEATH(loop->MakeNoArgWatcher<TestMessage>("/test", [&]() {}), "/test");
}

// Verify that SetRuntimeRealtimePriority fails while running.
TEST_P(AbstractEventLoopDeathTest, SetRuntimeRealtimePriority) {
  auto loop = MakePrimary();
  // Confirm that runtime priority calls work when not realtime.
  loop->SetRuntimeRealtimePriority(5);

  loop->OnRun([&]() { loop->SetRuntimeRealtimePriority(5); });

  EXPECT_DEATH(Run(), "realtime");
}

// Verify that SetRuntimeAffinity fails while running.
TEST_P(AbstractEventLoopDeathTest, SetRuntimeAffinity) {
  auto loop = MakePrimary();
  // Confirm that runtime priority calls work when not running.
  loop->SetRuntimeAffinity(MakeCpusetFromCpus({0}));

  loop->OnRun([&]() { loop->SetRuntimeAffinity(MakeCpusetFromCpus({1})); });

  EXPECT_DEATH(Run(), "Cannot set affinity while running");
}

// Verify that registering a watcher and a sender for "/test" fails.
TEST_P(AbstractEventLoopDeathTest, WatcherAndSender) {
  auto loop = Make();
  auto sender = loop->MakeSender<TestMessage>("/test");
  EXPECT_DEATH(loop->MakeWatcher("/test", [&](const TestMessage &) {}),
               "/test");
}

// Verify that creating too many senders fails.
TEST_P(AbstractEventLoopDeathTest, TooManySenders) {
  auto loop = Make();
  std::vector<aos::Sender<TestMessage>> senders;
  for (int i = 0; i < 10; ++i) {
    senders.emplace_back(loop->MakeSender<TestMessage>("/test"));
  }
  EXPECT_DEATH({ loop->MakeSender<TestMessage>("/test"); },
               "Failed to create sender on \\{ \"name\": \"/test\", \"type\": "
               "\"aos.TestMessage\"[^}]*\\ }, too many senders.");
}

// Verify that creating too many fetchers fails.
TEST_P(AbstractEventLoopDeathTest, TooManyFetchers) {
  if (read_method() != ReadMethod::PIN) {
    // Other read methods don't limit the number of readers, so just skip this.
    return;
  }

  auto loop = Make();
  std::vector<aos::Fetcher<TestMessage>> fetchers;
  for (int i = 0; i < 10; ++i) {
    fetchers.emplace_back(loop->MakeFetcher<TestMessage>("/test"));
  }
  EXPECT_DEATH({ loop->MakeFetcher<TestMessage>("/test"); },
               "Failed to create reader on \\{ \"name\": \"/test\", \"type\": "
               "\"aos.TestMessage\"[^}]*\\ }, too many readers.");
}

// Verify that creating too many fetchers, split between two event loops, fails.
TEST_P(AbstractEventLoopDeathTest, TooManyFetchersTwoLoops) {
  if (read_method() != ReadMethod::PIN) {
    // Other read methods don't limit the number of readers, so just skip this.
    return;
  }

  auto loop = Make();
  auto loop2 = Make();
  std::vector<aos::Fetcher<TestMessage>> fetchers;
  for (int i = 0; i < 5; ++i) {
    fetchers.emplace_back(loop->MakeFetcher<TestMessage>("/test"));
    fetchers.emplace_back(loop2->MakeFetcher<TestMessage>("/test"));
  }
  EXPECT_DEATH({ loop->MakeFetcher<TestMessage>("/test"); },
               "Failed to create reader on \\{ \"name\": \"/test\", \"type\": "
               "\"aos.TestMessage\"[^}]*\\ }, too many readers.");
}

// Verify that creating too many watchers fails.
TEST_P(AbstractEventLoopDeathTest, TooManyWatchers) {
  if (read_method() != ReadMethod::PIN) {
    // Other read methods don't limit the number of readers, so just skip this.
    return;
  }

  std::vector<std::unique_ptr<EventLoop>> loops;
  for (int i = 0; i < 10; ++i) {
    loops.emplace_back(Make());
    loops.back()->MakeWatcher("/test", [](const TestMessage &) {});
  }
  EXPECT_DEATH({ Make()->MakeWatcher("/test", [](const TestMessage &) {}); },
               "Failed to create reader on \\{ \"name\": \"/test\", \"type\": "
               "\"aos.TestMessage\"[^}]*\\ }, too many readers.");
}

// Verify that creating too many watchers and fetchers combined fails.
TEST_P(AbstractEventLoopDeathTest, TooManyWatchersAndFetchers) {
  if (read_method() != ReadMethod::PIN) {
    // Other read methods don't limit the number of readers, so just skip this.
    return;
  }

  auto loop = Make();
  std::vector<aos::Fetcher<TestMessage>> fetchers;
  std::vector<std::unique_ptr<EventLoop>> loops;
  for (int i = 0; i < 5; ++i) {
    fetchers.emplace_back(loop->MakeFetcher<TestMessage>("/test"));
    loops.emplace_back(Make());
    loops.back()->MakeWatcher("/test", [](const TestMessage &) {});
  }
  EXPECT_DEATH({ loop->MakeFetcher<TestMessage>("/test"); },
               "Failed to create reader on \\{ \"name\": \"/test\", \"type\": "
               "\"aos.TestMessage\"[^}]*\\ }, too many readers.");
}

// Verify that we can't create a sender inside OnRun.
TEST_P(AbstractEventLoopDeathTest, SenderInOnRun) {
  auto loop1 = MakePrimary();

  loop1->OnRun(
      [&]() { auto sender = loop1->MakeSender<TestMessage>("/test2"); });

  EXPECT_DEATH(Run(), "running");
}

// Verify that we can't create a watcher inside OnRun.
TEST_P(AbstractEventLoopDeathTest, WatcherInOnRun) {
  auto loop1 = MakePrimary();

  loop1->OnRun(
      [&]() { loop1->MakeWatcher("/test", [&](const TestMessage &) {}); });

  EXPECT_DEATH(Run(), "running");
}

// Verify that we can't create a no-arg watcher inside OnRun.
TEST_P(AbstractEventLoopDeathTest, NoArgWatcherInOnRun) {
  auto loop1 = MakePrimary();

  loop1->OnRun(
      [&]() { loop1->MakeNoArgWatcher<TestMessage>("/test", [&]() {}); });

  EXPECT_DEATH(Run(), "running");
}

// Verify that Quit() works when there are multiple watchers.
TEST_P(AbstractEventLoopTest, MultipleWatcherQuit) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  loop2->MakeWatcher("/test1", [&](const TestMessage &) {});
  loop2->MakeWatcher("/test2", [&](const TestMessage &message) {
    EXPECT_EQ(message.value(), 200);
    this->Exit();
  });

  auto sender = loop1->MakeSender<TestMessage>("/test2");

  loop2->OnRun([&]() {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  Run();
}

// Verify that timer intervals and duration function properly.
TEST_P(AbstractEventLoopTest, TimerIntervalAndDuration) {
  // Force a slower rate so we are guarenteed to have reports for our timer.
  FLAGS_timing_report_ms = 2000;

  const int kCount = 5;

  auto loop = MakePrimary();
  auto loop2 = Make();

  ::std::vector<::aos::monotonic_clock::time_point> times;
  ::std::vector<::aos::monotonic_clock::time_point> expected_times;

  Fetcher<timing::Report> report_fetcher =
      loop2->MakeFetcher<timing::Report>("/aos");
  EXPECT_FALSE(report_fetcher.Fetch());

  auto test_timer = loop->AddTimer([this, &times, &expected_times, &loop]() {
    times.push_back(loop->monotonic_now());
    EXPECT_EQ(loop->context().monotonic_remote_time, monotonic_clock::min_time);
    EXPECT_EQ(loop->context().realtime_event_time, realtime_clock::min_time);
    EXPECT_EQ(loop->context().realtime_remote_time, realtime_clock::min_time);
    EXPECT_EQ(loop->context().queue_index, 0xffffffffu);
    EXPECT_EQ(loop->context().size, 0u);
    EXPECT_EQ(loop->context().data, nullptr);
    EXPECT_EQ(loop->context().buffer_index, -1);

    expected_times.push_back(loop->context().monotonic_event_time);
    if (times.size() == kCount) {
      this->Exit();
    }
  });
  test_timer->set_name("Test loop");

  const monotonic_clock::time_point start_time = loop->monotonic_now();
  // TODO(austin): This should be an error...  Should be done in OnRun only.
  test_timer->Setup(start_time + chrono::seconds(1), chrono::seconds(1));

  Run();

  // Confirm that we got both the right number of samples, and it's odd.
  EXPECT_EQ(times.size(), static_cast<size_t>(kCount));
  EXPECT_EQ(times.size(), expected_times.size());
  EXPECT_EQ((times.size() % 2), 1);

  // Grab the middle sample.
  ::aos::monotonic_clock::time_point average_time = times[times.size() / 2];

  // Add up all the delays of all the times.
  ::aos::monotonic_clock::duration sum = chrono::seconds(0);
  for (const ::aos::monotonic_clock::time_point time : times) {
    sum += time - average_time;
  }

  // Average and add to the middle to find the average time.
  sum /= times.size();
  average_time += sum;

  // Compute the offset from the average and the expected average.  It
  // should be pretty close to 0.
  const ::aos::monotonic_clock::duration remainder =
      average_time - start_time - chrono::seconds(times.size() / 2 + 1);

  const chrono::milliseconds kEpsilon(100);
  EXPECT_LT(remainder, +kEpsilon);
  EXPECT_GT(remainder, -kEpsilon);

  // Make sure that the average duration is close to 1 second.
  EXPECT_NEAR(chrono::duration_cast<chrono::duration<double>>(times.back() -
                                                              times.front())
                      .count() /
                  static_cast<double>(times.size() - 1),
              1.0, 0.1);

  // Confirm that the ideal wakeup times increment correctly.
  for (size_t i = 1; i < expected_times.size(); ++i) {
    EXPECT_EQ(expected_times[i], expected_times[i - 1] + chrono::seconds(1));
  }

  for (size_t i = 0; i < expected_times.size(); ++i) {
    EXPECT_EQ((expected_times[i] - start_time) % chrono::seconds(1),
              chrono::seconds(0));
  }

  EXPECT_LT(expected_times[expected_times.size() / 2], average_time + kEpsilon);
  EXPECT_GT(expected_times[expected_times.size() / 2], average_time - kEpsilon);

  // And, since we are here, check that the timing report makes sense.
  // Start by looking for our event loop's timing.
  FlatbufferDetachedBuffer<timing::Report> report =
      FlatbufferDetachedBuffer<timing::Report>::Empty();
  while (report_fetcher.FetchNext()) {
    if (report_fetcher->name()->string_view() == "primary") {
      report = CopyFlatBuffer(report_fetcher.get());
    }
  }

  // Confirm that we have the right number of reports, and the contents are
  // sane.
  VLOG(1) << FlatbufferToJson(report, {.multi_line = true});

  EXPECT_EQ(report.message().name()->string_view(), "primary");

  ASSERT_NE(report.message().senders(), nullptr);
  EXPECT_EQ(report.message().senders()->size(), 2);

  ASSERT_NE(report.message().timers(), nullptr);
  EXPECT_EQ(report.message().timers()->size(), 2);

  EXPECT_EQ(report.message().timers()->Get(0)->name()->string_view(),
            "Test loop");
  EXPECT_GE(report.message().timers()->Get(0)->count(), 1);

  EXPECT_EQ(report.message().timers()->Get(1)->name()->string_view(),
            "timing_reports");
  EXPECT_EQ(report.message().timers()->Get(1)->count(), 1);

  // Make sure there is a single phased loop report with our report in it.
  ASSERT_EQ(report.message().phased_loops(), nullptr);
}

// Verify that we can change a timer's parameters during execution.
TEST_P(AbstractEventLoopTest, TimerChangeParameters) {
  auto loop = MakePrimary();
  ::std::vector<::aos::monotonic_clock::time_point> iteration_list;

  auto test_timer = loop->AddTimer([&iteration_list, &loop]() {
    iteration_list.push_back(loop->monotonic_now());
  });

  auto modifier_timer = loop->AddTimer([&loop, &test_timer]() {
    test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(30));
  });

  test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(20));
  modifier_timer->Setup(loop->monotonic_now() +
                        ::std::chrono::milliseconds(45));
  EndEventLoop(loop.get(), ::std::chrono::milliseconds(150));
  Run();

  EXPECT_EQ(iteration_list.size(), 7);
}

// Verify that we can disable a timer during execution.
TEST_P(AbstractEventLoopTest, TimerDisable) {
  auto loop = MakePrimary();
  ::std::vector<::aos::monotonic_clock::time_point> iteration_list;

  auto test_timer = loop->AddTimer([&iteration_list, &loop]() {
    iteration_list.push_back(loop->monotonic_now());
  });

  auto ender_timer = loop->AddTimer([&test_timer]() { test_timer->Disable(); });

  test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(20));
  ender_timer->Setup(loop->monotonic_now() + ::std::chrono::milliseconds(45));
  EndEventLoop(loop.get(), ::std::chrono::milliseconds(150));
  Run();

  EXPECT_EQ(iteration_list.size(), 3);
}

// Verify that we can disable a timer during execution of another timer
// scheduled for the same time, with one ordering of creation for the timers.
//
// Also schedule some more events to reshuffle the heap in EventLoop used for
// tracking events to change up the order. This used to segfault
// SimulatedEventLoop.
TEST_P(AbstractEventLoopTest, TimerDisableOther) {
  for (bool creation_order : {true, false}) {
    for (bool setup_order : {true, false}) {
      for (int shuffle_events = 0; shuffle_events < 5; ++shuffle_events) {
        auto loop = MakePrimary();
        aos::TimerHandler *test_timer, *ender_timer;
        if (creation_order) {
          test_timer = loop->AddTimer([]() {});
          ender_timer =
              loop->AddTimer([&test_timer]() { test_timer->Disable(); });
        } else {
          ender_timer =
              loop->AddTimer([&test_timer]() { test_timer->Disable(); });
          test_timer = loop->AddTimer([]() {});
        }

        const auto start = loop->monotonic_now();

        for (int i = 0; i < shuffle_events; ++i) {
          loop->AddTimer([]() {})->Setup(start + std::chrono::milliseconds(10));
        }

        if (setup_order) {
          test_timer->Setup(start + ::std::chrono::milliseconds(20));
          ender_timer->Setup(start + ::std::chrono::milliseconds(20));
        } else {
          ender_timer->Setup(start + ::std::chrono::milliseconds(20));
          test_timer->Setup(start + ::std::chrono::milliseconds(20));
        }
        EndEventLoop(loop.get(), ::std::chrono::milliseconds(40));
        Run();
      }
    }
  }
}

// Verifies that the event loop implementations detect when Channel is not a
// pointer into confguration()
TEST_P(AbstractEventLoopDeathTest, InvalidChannel) {
  auto loop = MakePrimary();

  const Channel *channel = configuration::GetChannel(
      loop->configuration(), "/test", "aos.TestMessage", "", nullptr);

  FlatbufferDetachedBuffer<Channel> channel_copy = CopyFlatBuffer(channel);

  EXPECT_DEATH(
      { loop->MakeRawSender(&channel_copy.message()); },
      "Channel pointer not found in configuration\\(\\)->channels\\(\\)");

  EXPECT_DEATH(
      { loop->MakeRawFetcher(&channel_copy.message()); },
      "Channel pointer not found in configuration\\(\\)->channels\\(\\)");

  EXPECT_DEATH(
      {
        loop->MakeRawWatcher(&channel_copy.message(),
                             [](const Context, const void *) {});
      },
      "Channel pointer not found in configuration\\(\\)->channels\\(\\)");
}

// Verifies that the event loop implementations detect when Channel has an
// invalid alignment.
TEST_P(AbstractEventLoopDeathTest, InvalidChannelAlignment) {
  const char *const kError = "multiple of alignment";
  InvalidChannelAlignment();

  auto loop = MakePrimary();

  const Channel *channel = configuration::GetChannel(
      loop->configuration(), "/test", "aos.TestMessage", "", nullptr);

  EXPECT_DEATH({ loop->MakeRawSender(channel); }, kError);
  EXPECT_DEATH({ loop->MakeSender<TestMessage>("/test"); }, kError);

  EXPECT_DEATH({ loop->MakeRawFetcher(channel); }, kError);
  EXPECT_DEATH({ loop->MakeFetcher<TestMessage>("/test"); }, kError);

  EXPECT_DEATH(
      { loop->MakeRawWatcher(channel, [](const Context &, const void *) {}); },
      kError);
  EXPECT_DEATH({ loop->MakeRawNoArgWatcher(channel, [](const Context &) {}); },
               kError);

  EXPECT_DEATH({ loop->MakeNoArgWatcher<TestMessage>("/test", []() {}); },
               kError);
  EXPECT_DEATH({ loop->MakeWatcher("/test", [](const TestMessage &) {}); },
               kError);
}

// Verify that the send time on a message is roughly right when using a watcher.
TEST_P(AbstractEventLoopTest, MessageSendTime) {
  auto loop1 = MakePrimary();
  auto loop2 = Make();
  auto sender = loop2->MakeSender<TestMessage>("/test");
  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  auto test_timer = loop1->AddTimer([&sender]() {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  bool triggered = false;
  loop1->MakeWatcher("/test", [&](const TestMessage &msg) {
    // Confirm that the data pointer makes sense from a watcher, and all the
    // timestamps look right.
    EXPECT_GT(&msg, loop1->context().data);
    EXPECT_EQ(loop1->context().monotonic_remote_time,
              loop1->context().monotonic_event_time);
    EXPECT_EQ(loop1->context().realtime_remote_time,
              loop1->context().realtime_event_time);

    const aos::monotonic_clock::time_point monotonic_now =
        loop1->monotonic_now();
    const aos::realtime_clock::time_point realtime_now = loop1->realtime_now();

    EXPECT_LE(loop1->context().monotonic_event_time, monotonic_now);
    EXPECT_LE(loop1->context().realtime_event_time, realtime_now);
    EXPECT_GE(loop1->context().monotonic_event_time + chrono::milliseconds(500),
              monotonic_now);
    EXPECT_GE(loop1->context().realtime_event_time + chrono::milliseconds(500),
              realtime_now);

    EXPECT_LT(&msg, reinterpret_cast<const void *>(
                        reinterpret_cast<const char *>(loop1->context().data) +
                        loop1->context().size));
    if (read_method() == ReadMethod::PIN) {
      EXPECT_GE(loop1->context().buffer_index, 0);
      EXPECT_LT(loop1->context().buffer_index,
                loop1->NumberBuffers(
                    configuration::GetChannel(loop1->configuration(), "/test",
                                              "aos.TestMessage", "", nullptr)));
    } else {
      EXPECT_EQ(-1, loop1->context().buffer_index);
    }
    triggered = true;
  });

  test_timer->Setup(loop1->monotonic_now() + ::std::chrono::seconds(1));

  EndEventLoop(loop1.get(), ::std::chrono::seconds(2));
  Run();

  EXPECT_TRUE(triggered);

  ASSERT_TRUE(fetcher.Fetch());

  monotonic_clock::duration monotonic_time_offset =
      fetcher.context().monotonic_event_time -
      (loop1->monotonic_now() - ::std::chrono::seconds(1));
  realtime_clock::duration realtime_time_offset =
      fetcher.context().realtime_event_time -
      (loop1->realtime_now() - ::std::chrono::seconds(1));

  EXPECT_EQ(fetcher.context().realtime_event_time,
            fetcher.context().realtime_remote_time);
  EXPECT_EQ(fetcher.context().monotonic_event_time,
            fetcher.context().monotonic_remote_time);

  EXPECT_TRUE(monotonic_time_offset > ::std::chrono::milliseconds(-500))
      << ": Got "
      << fetcher.context().monotonic_event_time.time_since_epoch().count()
      << " expected " << loop1->monotonic_now().time_since_epoch().count();
  // Confirm that the data pointer makes sense.
  EXPECT_GT(fetcher.get(), fetcher.context().data);
  EXPECT_LT(fetcher.get(),
            reinterpret_cast<const void *>(
                reinterpret_cast<const char *>(fetcher.context().data) +
                fetcher.context().size));
  EXPECT_TRUE(monotonic_time_offset < ::std::chrono::milliseconds(500))
      << ": Got "
      << fetcher.context().monotonic_event_time.time_since_epoch().count()
      << " expected " << loop1->monotonic_now().time_since_epoch().count();

  EXPECT_TRUE(realtime_time_offset > ::std::chrono::milliseconds(-500))
      << ": Got "
      << fetcher.context().realtime_event_time.time_since_epoch().count()
      << " expected " << loop1->realtime_now().time_since_epoch().count();
  EXPECT_TRUE(realtime_time_offset < ::std::chrono::milliseconds(500))
      << ": Got "
      << fetcher.context().realtime_event_time.time_since_epoch().count()
      << " expected " << loop1->realtime_now().time_since_epoch().count();
}

// Verify that the send time on a message is roughly right when using a no-arg
// watcher. To get a message, we need to use a fetcher to actually access the
// message. This is also the main use case for no-arg fetchers.
TEST_P(AbstractEventLoopTest, MessageSendTimeNoArg) {
  auto loop1 = MakePrimary();
  auto loop2 = Make();
  auto sender = loop2->MakeSender<TestMessage>("/test");
  auto fetcher = loop1->MakeFetcher<TestMessage>("/test");

  auto test_timer = loop1->AddTimer([&sender]() {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  bool triggered = false;
  loop1->MakeNoArgWatcher<TestMessage>("/test", [&]() {
    // Confirm that we can indeed use a fetcher on this channel from this
    // context, and it results in a sane data pointer and timestamps.
    ASSERT_TRUE(fetcher.Fetch());

    EXPECT_EQ(loop1->context().monotonic_remote_time,
              loop1->context().monotonic_event_time);
    EXPECT_EQ(loop1->context().realtime_remote_time,
              loop1->context().realtime_event_time);

    const aos::monotonic_clock::time_point monotonic_now =
        loop1->monotonic_now();
    const aos::realtime_clock::time_point realtime_now = loop1->realtime_now();

    EXPECT_LE(loop1->context().monotonic_event_time, monotonic_now);
    EXPECT_LE(loop1->context().realtime_event_time, realtime_now);
    EXPECT_GE(loop1->context().monotonic_event_time + chrono::milliseconds(500),
              monotonic_now);
    EXPECT_GE(loop1->context().realtime_event_time + chrono::milliseconds(500),
              realtime_now);

    triggered = true;
  });

  test_timer->Setup(loop1->monotonic_now() + ::std::chrono::seconds(1));

  EndEventLoop(loop1.get(), ::std::chrono::seconds(2));
  Run();

  ASSERT_TRUE(triggered);

  monotonic_clock::duration monotonic_time_offset =
      fetcher.context().monotonic_event_time -
      (loop1->monotonic_now() - ::std::chrono::seconds(1));
  realtime_clock::duration realtime_time_offset =
      fetcher.context().realtime_event_time -
      (loop1->realtime_now() - ::std::chrono::seconds(1));

  EXPECT_EQ(fetcher.context().realtime_event_time,
            fetcher.context().realtime_remote_time);
  EXPECT_EQ(fetcher.context().monotonic_event_time,
            fetcher.context().monotonic_remote_time);

  EXPECT_TRUE(monotonic_time_offset > ::std::chrono::milliseconds(-500))
      << ": Got "
      << fetcher.context().monotonic_event_time.time_since_epoch().count()
      << " expected " << loop1->monotonic_now().time_since_epoch().count();
  // Confirm that the data pointer makes sense.
  EXPECT_GT(fetcher.get(), fetcher.context().data);
  EXPECT_LT(fetcher.get(),
            reinterpret_cast<const void *>(
                reinterpret_cast<const char *>(fetcher.context().data) +
                fetcher.context().size));
  EXPECT_TRUE(monotonic_time_offset < ::std::chrono::milliseconds(500))
      << ": Got "
      << fetcher.context().monotonic_event_time.time_since_epoch().count()
      << " expected " << loop1->monotonic_now().time_since_epoch().count();

  EXPECT_TRUE(realtime_time_offset > ::std::chrono::milliseconds(-500))
      << ": Got "
      << fetcher.context().realtime_event_time.time_since_epoch().count()
      << " expected " << loop1->realtime_now().time_since_epoch().count();
  EXPECT_TRUE(realtime_time_offset < ::std::chrono::milliseconds(500))
      << ": Got "
      << fetcher.context().realtime_event_time.time_since_epoch().count()
      << " expected " << loop1->realtime_now().time_since_epoch().count();
}

// Tests that a couple phased loops run in a row result in the correct offset
// and period.
TEST_P(AbstractEventLoopTest, PhasedLoopTest) {
  // Force a slower rate so we are guarenteed to have reports for our phased
  // loop.
  FLAGS_timing_report_ms = 2000;

  const chrono::milliseconds kOffset = chrono::milliseconds(400);
  const int kCount = 5;

  auto loop1 = MakePrimary();
  auto loop2 = Make();

  Fetcher<timing::Report> report_fetcher =
      loop2->MakeFetcher<timing::Report>("/aos");
  EXPECT_FALSE(report_fetcher.Fetch());

  // Collect up a couple of samples.
  ::std::vector<::aos::monotonic_clock::time_point> times;
  ::std::vector<::aos::monotonic_clock::time_point> expected_times;

  // Run kCount iterations.
  loop1
      ->AddPhasedLoop(
          [&times, &expected_times, &loop1, this](int count) {
            EXPECT_EQ(count, 1);
            times.push_back(loop1->monotonic_now());
            expected_times.push_back(loop1->context().monotonic_event_time);

            EXPECT_EQ(loop1->context().monotonic_remote_time,
                      monotonic_clock::min_time);
            EXPECT_EQ(loop1->context().realtime_event_time,
                      realtime_clock::min_time);
            EXPECT_EQ(loop1->context().realtime_remote_time,
                      realtime_clock::min_time);
            EXPECT_EQ(loop1->context().queue_index, 0xffffffffu);
            EXPECT_EQ(loop1->context().size, 0u);
            EXPECT_EQ(loop1->context().data, nullptr);
            EXPECT_EQ(loop1->context().buffer_index, -1);

            if (times.size() == kCount) {
              LOG(INFO) << "Exiting";
              this->Exit();
            }
          },
          chrono::seconds(1), kOffset)
      ->set_name("Test loop");

  // Add a delay to make sure that delay during startup doesn't result in a
  // "missed cycle".
  SleepFor(chrono::seconds(2));

  Run();

  // Confirm that we got both the right number of samples, and it's odd.
  EXPECT_EQ(times.size(), static_cast<size_t>(kCount));
  EXPECT_EQ(times.size(), expected_times.size());
  EXPECT_EQ((times.size() % 2), 1);

  // Grab the middle sample.
  ::aos::monotonic_clock::time_point average_time = times[times.size() / 2];

  // Add up all the delays of all the times.
  ::aos::monotonic_clock::duration sum = chrono::seconds(0);
  for (const ::aos::monotonic_clock::time_point time : times) {
    sum += time - average_time;
  }

  // Average and add to the middle to find the average time.
  sum /= times.size();
  average_time += sum;

  // Compute the offset from the start of the second of the average time.  This
  // should be pretty close to the offset.
  const ::aos::monotonic_clock::duration remainder =
      average_time.time_since_epoch() -
      chrono::duration_cast<chrono::seconds>(average_time.time_since_epoch());

  const chrono::milliseconds kEpsilon(100);
  EXPECT_LT(remainder, kOffset + kEpsilon);
  EXPECT_GT(remainder, kOffset - kEpsilon);

  // Make sure that the average duration is close to 1 second.
  EXPECT_NEAR(chrono::duration_cast<chrono::duration<double>>(times.back() -
                                                              times.front())
                      .count() /
                  static_cast<double>(times.size() - 1),
              1.0, 0.1);

  // Confirm that the ideal wakeup times increment correctly.
  for (size_t i = 1; i < expected_times.size(); ++i) {
    EXPECT_EQ(expected_times[i], expected_times[i - 1] + chrono::seconds(1));
  }

  for (size_t i = 0; i < expected_times.size(); ++i) {
    EXPECT_EQ(expected_times[i].time_since_epoch() % chrono::seconds(1),
              kOffset);
  }

  EXPECT_LT(expected_times[expected_times.size() / 2], average_time + kEpsilon);
  EXPECT_GT(expected_times[expected_times.size() / 2], average_time - kEpsilon);

  // And, since we are here, check that the timing report makes sense.
  // Start by looking for our event loop's timing.
  FlatbufferDetachedBuffer<timing::Report> report =
      FlatbufferDetachedBuffer<timing::Report>::Empty();
  while (report_fetcher.FetchNext()) {
    if (report_fetcher->name()->string_view() == "primary") {
      report = CopyFlatBuffer(report_fetcher.get());
    }
  }

  VLOG(1) << FlatbufferToJson(report, {.multi_line = true});

  EXPECT_EQ(report.message().name()->string_view(), "primary");

  ASSERT_NE(report.message().senders(), nullptr);
  EXPECT_EQ(report.message().senders()->size(), 2);

  ASSERT_NE(report.message().timers(), nullptr);
  EXPECT_EQ(report.message().timers()->size(), 1);

  // Make sure there is a single phased loop report with our report in it.
  ASSERT_NE(report.message().phased_loops(), nullptr);
  ASSERT_EQ(report.message().phased_loops()->size(), 1);
  EXPECT_EQ(report.message().phased_loops()->Get(0)->name()->string_view(),
            "Test loop");
  EXPECT_GE(report.message().phased_loops()->Get(0)->count(), 1);
}

// Tests that senders count correctly in the timing report.
TEST_P(AbstractEventLoopTest, SenderTimingReport) {
  FLAGS_timing_report_ms = 1000;
  auto loop1 = MakePrimary();

  auto loop2 = Make("watcher_loop");
  loop2->MakeWatcher("/test", [](const TestMessage &) {});

  auto loop3 = Make();

  Fetcher<timing::Report> report_fetcher =
      loop3->MakeFetcher<timing::Report>("/aos");
  EXPECT_FALSE(report_fetcher.Fetch());

  auto sender = loop1->MakeSender<TestMessage>("/test");

  // Add a timer to actually quit.
  auto test_timer = loop1->AddTimer([&sender]() {
    for (int i = 0; i < 10; ++i) {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200 + i);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
  });

  // Quit after 1 timing report, mid way through the next cycle.
  EndEventLoop(loop1.get(), chrono::milliseconds(2500));

  loop1->OnRun([&test_timer, &loop1]() {
    test_timer->Setup(loop1->monotonic_now() + chrono::milliseconds(1500));
  });

  Run();

  // And, since we are here, check that the timing report makes sense.
  // Start by looking for our event loop's timing.
  FlatbufferDetachedBuffer<timing::Report> primary_report =
      FlatbufferDetachedBuffer<timing::Report>::Empty();
  while (report_fetcher.FetchNext()) {
    LOG(INFO) << "Report " << FlatbufferToJson(report_fetcher.get());
    if (report_fetcher->name()->string_view() == "primary") {
      primary_report = CopyFlatBuffer(report_fetcher.get());
    }
  }

  LOG(INFO) << FlatbufferToJson(primary_report, {.multi_line = true});

  EXPECT_EQ(primary_report.message().name()->string_view(), "primary");

  ASSERT_NE(primary_report.message().senders(), nullptr);
  EXPECT_EQ(primary_report.message().senders()->size(), 3);

  // Confirm that the sender looks sane.
  EXPECT_EQ(
      loop1->configuration()
          ->channels()
          ->Get(primary_report.message().senders()->Get(0)->channel_index())
          ->name()
          ->string_view(),
      "/test");
  EXPECT_EQ(primary_report.message().senders()->Get(0)->count(), 10);

  // Confirm that the timing primary_report sender looks sane.
  EXPECT_EQ(
      loop1->configuration()
          ->channels()
          ->Get(primary_report.message().senders()->Get(1)->channel_index())
          ->name()
          ->string_view(),
      "/aos");
  EXPECT_EQ(primary_report.message().senders()->Get(1)->count(), 1);

  ASSERT_NE(primary_report.message().timers(), nullptr);
  EXPECT_EQ(primary_report.message().timers()->size(), 3);

  // Make sure there are no phased loops or watchers.
  ASSERT_EQ(primary_report.message().phased_loops(), nullptr);
  ASSERT_EQ(primary_report.message().watchers(), nullptr);
}

// Tests that senders count correctly in the timing report.
TEST_P(AbstractEventLoopTest, WatcherTimingReport) {
  FLAGS_timing_report_ms = 1000;
  auto loop1 = MakePrimary();
  loop1->MakeWatcher("/test", [](const TestMessage &) {});

  auto loop2 = Make("sender_loop");

  auto loop3 = Make();

  Fetcher<timing::Report> report_fetcher =
      loop3->MakeFetcher<timing::Report>("/aos");
  EXPECT_FALSE(report_fetcher.Fetch());

  auto sender = loop2->MakeSender<TestMessage>("/test");

  // Add a timer to actually quit.
  auto test_timer = loop1->AddTimer([&sender]() {
    for (int i = 0; i < 10; ++i) {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200 + i);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
  });

  // Quit after 1 timing report, mid way through the next cycle.
  EndEventLoop(loop1.get(), chrono::milliseconds(2500));

  loop1->OnRun([&test_timer, &loop1]() {
    test_timer->Setup(loop1->monotonic_now() + chrono::milliseconds(1500));
  });

  Run();

  // And, since we are here, check that the timing report makes sense.
  // Start by looking for our event loop's timing.
  FlatbufferDetachedBuffer<timing::Report> primary_report =
      FlatbufferDetachedBuffer<timing::Report>::Empty();
  while (report_fetcher.FetchNext()) {
    LOG(INFO) << "Report " << FlatbufferToJson(report_fetcher.get());
    if (report_fetcher->name()->string_view() == "primary") {
      primary_report = CopyFlatBuffer(report_fetcher.get());
    }
  }

  // Check the watcher report.
  VLOG(1) << FlatbufferToJson(primary_report, {.multi_line = true});

  EXPECT_EQ(primary_report.message().name()->string_view(), "primary");

  // Just the timing report timer.
  ASSERT_NE(primary_report.message().timers(), nullptr);
  EXPECT_EQ(primary_report.message().timers()->size(), 3);

  // No phased loops
  ASSERT_EQ(primary_report.message().phased_loops(), nullptr);

  ASSERT_NE(primary_report.message().watchers(), nullptr);
  ASSERT_EQ(primary_report.message().watchers()->size(), 1);
  EXPECT_EQ(primary_report.message().watchers()->Get(0)->count(), 10);
}

// Tests that fetchers count correctly in the timing report.
TEST_P(AbstractEventLoopTest, FetcherTimingReport) {
  FLAGS_timing_report_ms = 1000;
  auto loop1 = MakePrimary();
  auto loop2 = Make("sender_loop");

  auto loop3 = Make();

  Fetcher<timing::Report> report_fetcher =
      loop3->MakeFetcher<timing::Report>("/aos");
  EXPECT_FALSE(report_fetcher.Fetch());

  auto sender = loop2->MakeSender<TestMessage>("/test");
  auto fetcher1 = loop1->MakeFetcher<TestMessage>("/test");
  auto fetcher2 = loop1->MakeFetcher<TestMessage>("/test");
  fetcher1.Fetch();
  fetcher2.Fetch();

  // Add a timer to actually quit.
  auto test_timer = loop1->AddTimer([&sender]() {
    for (int i = 0; i < 10; ++i) {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200 + i);
      ASSERT_TRUE(msg.Send(builder.Finish()));
    }
  });

  auto test_timer2 = loop1->AddTimer([&fetcher1, &fetcher2]() {
    fetcher1.Fetch();
    while (fetcher2.FetchNext()) {
    }
  });

  // Quit after 1 timing report, mid way through the next cycle.
  EndEventLoop(loop1.get(), chrono::milliseconds(2500));

  loop1->OnRun([test_timer, test_timer2, &loop1]() {
    test_timer->Setup(loop1->monotonic_now() + chrono::milliseconds(1400));
    test_timer2->Setup(loop1->monotonic_now() + chrono::milliseconds(1600));
  });

  Run();

  // And, since we are here, check that the timing report makes sense.
  // Start by looking for our event loop's timing.
  FlatbufferDetachedBuffer<timing::Report> primary_report =
      FlatbufferDetachedBuffer<timing::Report>::Empty();
  while (report_fetcher.FetchNext()) {
    if (report_fetcher->name()->string_view() == "primary") {
      primary_report = CopyFlatBuffer(report_fetcher.get());
    }
  }

  VLOG(1) << FlatbufferToJson(primary_report, {.multi_line = true});

  EXPECT_EQ(primary_report.message().name()->string_view(), "primary");

  ASSERT_NE(primary_report.message().senders(), nullptr);
  EXPECT_EQ(primary_report.message().senders()->size(), 2);

  ASSERT_NE(primary_report.message().timers(), nullptr);
  EXPECT_EQ(primary_report.message().timers()->size(), 4);

  // Make sure there are no phased loops or watchers.
  ASSERT_EQ(primary_report.message().phased_loops(), nullptr);
  ASSERT_EQ(primary_report.message().watchers(), nullptr);

  // Now look at the fetchrs.
  ASSERT_NE(primary_report.message().fetchers(), nullptr);
  ASSERT_EQ(primary_report.message().fetchers()->size(), 2);

  EXPECT_EQ(primary_report.message().fetchers()->Get(0)->count(), 1);
  EXPECT_GE(primary_report.message().fetchers()->Get(0)->latency()->average(),
            0.1);
  EXPECT_GE(primary_report.message().fetchers()->Get(0)->latency()->min(), 0.1);
  EXPECT_GE(primary_report.message().fetchers()->Get(0)->latency()->max(), 0.1);
  EXPECT_EQ(primary_report.message()
                .fetchers()
                ->Get(0)
                ->latency()
                ->standard_deviation(),
            0.0);

  EXPECT_EQ(primary_report.message().fetchers()->Get(1)->count(), 10);
}

// Tests that a raw watcher and raw fetcher can receive messages from a raw
// sender without messing up offsets.
TEST_P(AbstractEventLoopTest, RawBasic) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();
  auto loop3 = Make();

  const std::string kData("971 is the best");

  std::unique_ptr<aos::RawSender> sender =
      loop1->MakeRawSender(configuration::GetChannel(
          loop1->configuration(), "/test", "aos.TestMessage", "", nullptr));

  std::unique_ptr<aos::RawFetcher> fetcher =
      loop3->MakeRawFetcher(configuration::GetChannel(
          loop3->configuration(), "/test", "aos.TestMessage", "", nullptr));

  loop2->OnRun(
      [&]() { EXPECT_TRUE(sender->Send(kData.data(), kData.size())); });

  bool happened = false;
  loop2->MakeRawWatcher(
      configuration::GetChannel(loop2->configuration(), "/test",
                                "aos.TestMessage", "", nullptr),
      [this, &kData, &fetcher, &happened](const Context &context,
                                          const void *message) {
        happened = true;
        EXPECT_EQ(std::string_view(kData),
                  std::string_view(reinterpret_cast<const char *>(message),
                                   context.size));
        EXPECT_EQ(std::string_view(kData),
                  std::string_view(reinterpret_cast<const char *>(context.data),
                                   context.size));

        ASSERT_TRUE(fetcher->Fetch());

        EXPECT_EQ(std::string_view(kData),
                  std::string_view(
                      reinterpret_cast<const char *>(fetcher->context().data),
                      fetcher->context().size));

        this->Exit();
      });

  EXPECT_FALSE(happened);
  Run();
  EXPECT_TRUE(happened);
}

// Tests that a raw watcher and raw fetcher can receive messages from a raw
// sender with remote times filled out.
TEST_P(AbstractEventLoopTest, RawRemoteTimes) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();
  auto loop3 = Make();

  const std::string kData("971 is the best");

  const aos::monotonic_clock::time_point monotonic_remote_time =
      aos::monotonic_clock::time_point(chrono::seconds(1501));
  const aos::realtime_clock::time_point realtime_remote_time =
      aos::realtime_clock::time_point(chrono::seconds(3132));

  std::unique_ptr<aos::RawSender> sender =
      loop1->MakeRawSender(configuration::GetChannel(
          loop1->configuration(), "/test", "aos.TestMessage", "", nullptr));

  std::unique_ptr<aos::RawFetcher> fetcher =
      loop3->MakeRawFetcher(configuration::GetChannel(
          loop3->configuration(), "/test", "aos.TestMessage", "", nullptr));

  loop2->OnRun([&]() {
    EXPECT_TRUE(sender->Send(kData.data(), kData.size(), monotonic_remote_time,
                             realtime_remote_time));
  });

  bool happened = false;
  loop2->MakeRawWatcher(
      configuration::GetChannel(loop2->configuration(), "/test",
                                "aos.TestMessage", "", nullptr),
      [this, monotonic_remote_time, realtime_remote_time, &fetcher, &happened](
          const Context &context, const void * /*message*/) {
        happened = true;
        EXPECT_EQ(monotonic_remote_time, context.monotonic_remote_time);
        EXPECT_EQ(realtime_remote_time, context.realtime_remote_time);

        ASSERT_TRUE(fetcher->Fetch());
        EXPECT_EQ(monotonic_remote_time,
                  fetcher->context().monotonic_remote_time);
        EXPECT_EQ(realtime_remote_time,
                  fetcher->context().realtime_remote_time);

        this->Exit();
      });

  EXPECT_FALSE(happened);
  Run();
  EXPECT_TRUE(happened);
}

// Tests that a raw sender fills out sent data.
TEST_P(AbstractEventLoopTest, RawSenderSentData) {
  auto loop1 = MakePrimary();

  const std::string kData("971 is the best");

  std::unique_ptr<aos::RawSender> sender =
      loop1->MakeRawSender(configuration::GetChannel(
          loop1->configuration(), "/test", "aos.TestMessage", "", nullptr));

  const aos::monotonic_clock::time_point monotonic_now = loop1->monotonic_now();
  const aos::realtime_clock::time_point realtime_now = loop1->realtime_now();

  EXPECT_TRUE(sender->Send(kData.data(), kData.size()));

  EXPECT_GE(sender->monotonic_sent_time(), monotonic_now);
  EXPECT_LE(sender->monotonic_sent_time(),
            monotonic_now + chrono::milliseconds(100));
  EXPECT_GE(sender->realtime_sent_time(), realtime_now);
  EXPECT_LE(sender->realtime_sent_time(),
            realtime_now + chrono::milliseconds(100));
  EXPECT_EQ(sender->sent_queue_index(), 0u);

  EXPECT_TRUE(sender->Send(kData.data(), kData.size()));

  EXPECT_GE(sender->monotonic_sent_time(), monotonic_now);
  EXPECT_LE(sender->monotonic_sent_time(),
            monotonic_now + chrono::milliseconds(100));
  EXPECT_GE(sender->realtime_sent_time(), realtime_now);
  EXPECT_LE(sender->realtime_sent_time(),
            realtime_now + chrono::milliseconds(100));
  EXPECT_EQ(sender->sent_queue_index(), 1u);
}

// Tests that not setting up nodes results in no node.
TEST_P(AbstractEventLoopTest, NoNode) {
  auto loop1 = Make();
  auto loop2 = MakePrimary();

  EXPECT_EQ(loop1->node(), nullptr);
  EXPECT_EQ(loop2->node(), nullptr);
}

// Tests that setting up nodes results in node being set.
TEST_P(AbstractEventLoopTest, Node) {
  EnableNodes("me");

  auto loop1 = Make();
  auto loop2 = MakePrimary();

  EXPECT_NE(loop1->node(), nullptr);
  EXPECT_NE(loop2->node(), nullptr);
}

// Tests that watchers work with a node setup.
TEST_P(AbstractEventLoopTest, NodeWatcher) {
  EnableNodes("me");

  auto loop1 = Make();
  auto loop2 = Make();
  loop1->MakeWatcher("/test", [](const TestMessage &) {});
  loop2->MakeRawWatcher(
      configuration::GetChannel(configuration(), "/test", "aos.TestMessage", "",
                                nullptr),
      [](const Context &, const void *) {});
}

// Tests that no-arg watchers work with a node setup.
TEST_P(AbstractEventLoopTest, NodeNoArgWatcher) {
  EnableNodes("me");

  auto loop1 = Make();
  auto loop2 = Make();
  loop1->MakeWatcher("/test", [](const TestMessage &) {});
  loop2->MakeRawNoArgWatcher(
      configuration::GetChannel(configuration(), "/test", "aos.TestMessage", "",
                                nullptr),
      [](const Context &) {});
}

// Tests that fetcher work with a node setup.
TEST_P(AbstractEventLoopTest, NodeFetcher) {
  EnableNodes("me");
  auto loop1 = Make();

  auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
  auto raw_fetcher = loop1->MakeRawFetcher(configuration::GetChannel(
      configuration(), "/test", "aos.TestMessage", "", nullptr));
}

// Tests that sender work with a node setup.
TEST_P(AbstractEventLoopTest, NodeSender) {
  EnableNodes("me");
  auto loop1 = Make();

  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");
}

// Tests that watchers fail when created on the wrong node.
TEST_P(AbstractEventLoopDeathTest, NodeWatcher) {
  EnableNodes("them");

  auto loop1 = Make();
  auto loop2 = Make();
  EXPECT_DEATH({ loop1->MakeWatcher("/test", [](const TestMessage &) {}); },
               "node");
  EXPECT_DEATH(
      {
        loop2->MakeRawWatcher(
            configuration::GetChannel(configuration(), "/test",
                                      "aos.TestMessage", "", nullptr),
            [](const Context &, const void *) {});
      },
      "node");
  EXPECT_DEATH({ loop1->MakeNoArgWatcher<TestMessage>("/test", []() {}); },
               "node");
  EXPECT_DEATH(
      {
        loop2->MakeRawNoArgWatcher(
            configuration::GetChannel(configuration(), "/test",
                                      "aos.TestMessage", "", nullptr),
            [](const Context &) {});
      },
      "node");
}

// Tests that fetchers fail when created on the wrong node.
TEST_P(AbstractEventLoopDeathTest, NodeFetcher) {
  EnableNodes("them");
  auto loop1 = Make();

  EXPECT_DEATH({ auto fetcher = loop1->MakeFetcher<TestMessage>("/test"); },
               "node");
  EXPECT_DEATH(
      {
        auto raw_fetcher = loop1->MakeRawFetcher(configuration::GetChannel(
            configuration(), "/test", "aos.TestMessage", "", nullptr));
      },
      "node");
}

// Tests that senders fail when created on the wrong node.
TEST_P(AbstractEventLoopDeathTest, NodeSender) {
  EnableNodes("them");
  auto loop1 = Make();

  EXPECT_DEATH(
      {
        aos::Sender<TestMessage> sender =
            loop1->MakeSender<TestMessage>("/test");
      },
      "node");

  // Note: Creating raw senders is always supported.  Right now, this lets us
  // use them to create message_gateway.
}

// Tests creating multiple Builders from a single Sender at the same time.
TEST_P(AbstractEventLoopDeathTest, MultipleBuilders) {
  auto loop1 = Make();
  aos::Sender<TestMessage> sender = loop1->MakeSender<TestMessage>("/test");

  { auto builder = sender.MakeBuilder(); }
  {
    auto builder = sender.MakeBuilder();
    builder.MakeBuilder<TestMessage>().Finish();
  }
  {
    // Creating this after the first one was destroyed should be fine.
    auto builder = sender.MakeBuilder();
    builder.MakeBuilder<TestMessage>().Finish();
    // But not a second one.
    EXPECT_DEATH(sender.MakeBuilder().MakeBuilder<TestMessage>().Finish(),
                 "May not overwrite in-use allocator");
  }

  FlatbufferDetachedBuffer<TestMessage> detached =
      flatbuffers::DetachedBuffer();
  {
    auto builder = sender.MakeBuilder();
    detached = builder.Detach(builder.MakeBuilder<TestMessage>().Finish());
  }
  {
    // This is the second one, after the detached one, so it should fail.
    EXPECT_DEATH(sender.MakeBuilder().MakeBuilder<TestMessage>().Finish(),
                 "May not overwrite in-use allocator");
  }

  // Clear the detached one, and then we should be able to create another.
  detached = flatbuffers::DetachedBuffer();
  {
    auto builder = sender.MakeBuilder();
    builder.MakeBuilder<TestMessage>().Finish();
  }

  // And then detach another one.
  {
    auto builder = sender.MakeBuilder();
    detached = builder.Detach(builder.MakeBuilder<TestMessage>().Finish());
  }
}

// Tests sending a buffer detached from a different builder.
TEST_P(AbstractEventLoopDeathTest, WrongDetachedBuffer) {
  auto loop1 = Make();
  aos::Sender<TestMessage> sender1 = loop1->MakeSender<TestMessage>("/test");
  aos::Sender<TestMessage> sender2 = loop1->MakeSender<TestMessage>("/test");

  auto builder = sender1.MakeBuilder();
  FlatbufferDetachedBuffer<TestMessage> detached =
      builder.Detach(builder.MakeBuilder<TestMessage>().Finish());
  EXPECT_DEATH(sender2.SendDetached(std::move(detached)),
               "May only send the buffer detached from this Sender");
}

}  // namespace testing
}  // namespace aos
