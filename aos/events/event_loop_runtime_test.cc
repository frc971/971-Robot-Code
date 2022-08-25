#include "aos/events/event_loop_runtime_test_lib_rs_cxxgen.h"
#include "aos/events/ping_generated.h"
#include "aos/events/pong_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace aos::events::testing {
namespace {

template <typename F>
void MakeAndTestApplication(int value, F constructor) {
  const int32_t starting_count = completed_test_count();
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          aos::testing::ArtifactPath("aos/events/pingpong_config.json"));
  SimulatedEventLoopFactory factory{&config.message()};
  const auto ping_event_loop = factory.MakeEventLoop("ping");
  auto ping_sender = ping_event_loop->MakeSender<examples::Ping>("/test");
  auto pong_fetcher = ping_event_loop->MakeFetcher<examples::Pong>("/test");
  const auto rust_event_loop = factory.MakeEventLoop("pong");
  {
    auto test_application = constructor(rust_event_loop.get());
    int iteration = 0;
    ping_event_loop
        ->AddTimer([&]() {
          if (iteration++ > 0) {
            test_application->after_sending();
            factory.Exit();
            return;
          }
          test_application->before_sending();
          ASSERT_FALSE(pong_fetcher.Fetch());
          {
            auto builder = ping_sender.MakeBuilder();
            examples::Ping::Builder ping(*builder.fbb());
            ping.add_value(value);
            builder.CheckOk(builder.Send(ping.Finish()));
          }
        })
        ->Setup(
            ping_event_loop->monotonic_now() + std::chrono::milliseconds(10),
            std::chrono::milliseconds(10));
    ASSERT_EQ(starting_count, started_test_count());
    factory.Run();
    ASSERT_EQ(starting_count + 1, started_test_count());
    EXPECT_EQ(2, iteration);
  }
  ASSERT_EQ(starting_count + 1, completed_test_count());
  ASSERT_TRUE(pong_fetcher.Fetch());
  ASSERT_EQ(value, pong_fetcher->value());
}

}  // namespace

TEST(EventLoopRustTest, TestApplicationOnce) {
  MakeAndTestApplication(971, &make_test_application);
}

TEST(EventLoopRustTest, TestApplicationTwice) {
  MakeAndTestApplication(971, &make_test_application);
  MakeAndTestApplication(254, &make_test_application);
}

TEST(EventLoopRustTest, TestTypedApplicationTwice) {
  MakeAndTestApplication(971, &make_typed_test_application);
  MakeAndTestApplication(254, &make_typed_test_application);
}

}  // namespace aos::events::testing
