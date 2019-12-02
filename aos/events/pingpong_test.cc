#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

namespace chrono = std::chrono;

class PingPongTest : public ::testing::Test {
 public:
  PingPongTest()
      : config_(
            aos::configuration::ReadConfig("aos/events/pingpong_config.json")),
        event_loop_factory_(&config_.message()),
        ping_event_loop_(event_loop_factory_.MakeEventLoop("ping")),
        ping_(ping_event_loop_.get()),
        pong_event_loop_(event_loop_factory_.MakeEventLoop("pong")),
        pong_(pong_event_loop_.get()) {}

  // Config and factory.
  // The factory is a factory for connected event loops.  Each application needs
  // a separate event loop (because you can't send a message to yourself in a
  // single event loop).  The created event loops can then send messages to each
  // other and trigger callbacks to be called, or fetchers to receive data.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;

  // Event loop and app for Ping
  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;

  // Event loop and app for Pong
  std::unique_ptr<EventLoop> pong_event_loop_;
  Pong pong_;
};

// Tests that we can startup at all.  This confirms that the channels are all in
// the config.
TEST_F(PingPongTest, Starts) {
  // RunFor lives in the factory because all the loops need to run together, and
  // the factory is the only object that conceptually knows about all of the
  // loops at once.
  event_loop_factory_.RunFor(chrono::seconds(10));
}

// Tests that the number of pong messages matches the number of ping messages.
TEST_F(PingPongTest, AlwaysReplies) {
  std::unique_ptr<EventLoop> test_event_loop =
      event_loop_factory_.MakeEventLoop("test");

  int ping_count = 0;
  int pong_count = 0;

  // Confirm that the ping value matches.
  test_event_loop->MakeWatcher("/test",
                               [&ping_count](const examples::Ping &ping) {
                                 EXPECT_EQ(ping.value(), ping_count + 1);
                                 ++ping_count;
                               });
  // Confirm that the ping and pong counts both match, and the value also
  // matches.
  test_event_loop->MakeWatcher(
      "/test", [&pong_count, &ping_count](const examples::Pong &pong) {
        EXPECT_EQ(pong.value(), pong_count + 1);
        ++pong_count;
        EXPECT_EQ(ping_count, pong_count);
      });

  event_loop_factory_.RunFor(chrono::seconds(10));

  // We run at t=0 and t=10 seconds, which means we run 1 extra time.
  EXPECT_EQ(ping_count, 1001);
}

}  // namespace testing
}  // namespace aos
