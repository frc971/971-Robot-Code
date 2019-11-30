#include "aos/events/event_loop_param_test.h"

#include <chrono>

#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/test_message_generated.h"
#include "aos/flatbuffer_merge.h"
#include "glog/logging.h"

namespace aos {
namespace testing {
namespace {
namespace chrono = ::std::chrono;
}  // namespace

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

// Tests that a fetcher can fetch from a sender.
// Also tests that OnRun() works.
TEST_P(AbstractEventLoopTest, FetchWithoutRun) {
  auto loop1 = Make();
  auto loop2 = Make();
  auto loop3 = MakePrimary();

  auto sender = loop1->MakeSender<TestMessage>("/test");

  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  EXPECT_FALSE(fetcher.Fetch());

  aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
  TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
  builder.add_value(200);
  ASSERT_TRUE(msg.Send(builder.Finish()));

  EXPECT_TRUE(fetcher.Fetch());
  ASSERT_FALSE(fetcher.get() == nullptr);
  EXPECT_EQ(fetcher.get()->value(), 200);
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
}

// Verify that registering a watcher twice for "/test" fails.
TEST_P(AbstractEventLoopDeathTest, TwoWatcher) {
  auto loop = Make();
  loop->MakeWatcher("/test", [&](const TestMessage &) {});
  EXPECT_DEATH(loop->MakeWatcher("/test", [&](const TestMessage &) {}),
               "/test");
}

// Verify that SetRuntimeRealtimePriority fails while running.
TEST_P(AbstractEventLoopDeathTest, SetRuntimeRealtimePriority) {
  auto loop = MakePrimary();
  // Confirm that runtime priority calls work when not realtime.
  loop->SetRuntimeRealtimePriority(5);

  loop->OnRun([&]() { loop->SetRuntimeRealtimePriority(5); });

  EXPECT_DEATH(Run(), "realtime");
}

// Verify that registering a watcher and a sender for "/test" fails.
TEST_P(AbstractEventLoopDeathTest, WatcherAndSender) {
  auto loop = Make();
  auto sender = loop->MakeSender<TestMessage>("/test");
  EXPECT_DEATH(loop->MakeWatcher("/test", [&](const TestMessage &) {}),
               "/test");
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
  const int kCount = 5;

  auto loop = MakePrimary();
  ::std::vector<::aos::monotonic_clock::time_point> times;
  ::std::vector<::aos::monotonic_clock::time_point> expected_times;

  auto test_timer = loop->AddTimer([this, &times, &expected_times, &loop]() {
    times.push_back(loop->monotonic_now());
    expected_times.push_back(loop->context().monotonic_sent_time);
    if (times.size() == kCount) {
      this->Exit();
    }
  });

  monotonic_clock::time_point start_time = loop->monotonic_now();
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

  auto ender_timer = loop->AddTimer([&test_timer]() {
    test_timer->Disable();
  });

  test_timer->Setup(loop->monotonic_now(), ::std::chrono::milliseconds(20));
  ender_timer->Setup(loop->monotonic_now() +
                        ::std::chrono::milliseconds(45));
  EndEventLoop(loop.get(), ::std::chrono::milliseconds(150));
  Run();

  EXPECT_EQ(iteration_list.size(), 3);
}

// Verifies that the event loop implementations detect when Channel is not a
// pointer into confguration()
TEST_P(AbstractEventLoopDeathTest, InvalidChannel) {
  auto loop = MakePrimary();

  const Channel *channel = loop->configuration()->channels()->Get(0);

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

// Verify that the send time on a message is roughly right.
TEST_P(AbstractEventLoopTest, MessageSendTime) {
  auto loop1 = MakePrimary();
  auto loop2 = Make();
  auto sender = loop1->MakeSender<TestMessage>("/test");
  auto fetcher = loop2->MakeFetcher<TestMessage>("/test");

  auto test_timer = loop1->AddTimer([&sender]() {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    ASSERT_TRUE(msg.Send(builder.Finish()));
  });

  loop2->MakeWatcher("/test", [&loop2](const TestMessage &msg) {
    // Confirm that the data pointer makes sense from a watcher.
    EXPECT_GT(&msg, loop2->context().data);
    EXPECT_LT(&msg, reinterpret_cast<void *>(
                        reinterpret_cast<char *>(loop2->context().data) +
                        loop2->context().size));
  });

  test_timer->Setup(loop1->monotonic_now() + ::std::chrono::seconds(1));

  EndEventLoop(loop1.get(), ::std::chrono::seconds(2));
  Run();

  EXPECT_TRUE(fetcher.Fetch());

  monotonic_clock::duration monotonic_time_offset =
      fetcher.context().monotonic_sent_time -
      (loop1->monotonic_now() - ::std::chrono::seconds(1));
  realtime_clock::duration realtime_time_offset =
      fetcher.context().realtime_sent_time -
      (loop1->realtime_now() - ::std::chrono::seconds(1));

  EXPECT_TRUE(monotonic_time_offset > ::std::chrono::milliseconds(-500))
      << ": Got "
      << fetcher.context().monotonic_sent_time.time_since_epoch().count()
      << " expected " << loop1->monotonic_now().time_since_epoch().count();
  // Confirm that the data pointer makes sense.
  EXPECT_GT(fetcher.get(), fetcher.context().data);
  EXPECT_LT(fetcher.get(),
            reinterpret_cast<void *>(
                reinterpret_cast<char *>(fetcher.context().data) +
                fetcher.context().size));
  EXPECT_TRUE(monotonic_time_offset < ::std::chrono::milliseconds(500))
      << ": Got "
      << fetcher.context().monotonic_sent_time.time_since_epoch().count()
      << " expected " << loop1->monotonic_now().time_since_epoch().count();

  EXPECT_TRUE(realtime_time_offset > ::std::chrono::milliseconds(-500))
      << ": Got "
      << fetcher.context().realtime_sent_time.time_since_epoch().count()
      << " expected " << loop1->realtime_now().time_since_epoch().count();
  EXPECT_TRUE(realtime_time_offset < ::std::chrono::milliseconds(500))
      << ": Got "
      << fetcher.context().realtime_sent_time.time_since_epoch().count()
      << " expected " << loop1->realtime_now().time_since_epoch().count();
}

// Tests that a couple phased loops run in a row result in the correct offset
// and period.
TEST_P(AbstractEventLoopTest, PhasedLoopTest) {
  const chrono::milliseconds kOffset = chrono::milliseconds(400);
  const int kCount = 5;

  auto loop1 = MakePrimary();

  // Collect up a couple of samples.
  ::std::vector<::aos::monotonic_clock::time_point> times;
  ::std::vector<::aos::monotonic_clock::time_point> expected_times;

  // Run kCount iterations.
  loop1->AddPhasedLoop(
      [&times, &expected_times, &loop1, this](int count) {
        EXPECT_EQ(count, 1);
        times.push_back(loop1->monotonic_now());
        expected_times.push_back(loop1->context().monotonic_sent_time);
        if (times.size() == kCount) {
          this->Exit();
        }
      },
      chrono::seconds(1), kOffset);

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
}

}  // namespace testing
}  // namespace aos
