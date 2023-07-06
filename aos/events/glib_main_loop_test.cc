#include "aos/events/glib_main_loop.h"

#include <thread>

#include "glib-2.0/glib.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/testing/path.h"

namespace aos {
namespace testing {
using aos::testing::ArtifactPath;

const FlatbufferDetachedBuffer<Configuration> &Config() {
  static const FlatbufferDetachedBuffer<Configuration> result =
      configuration::ReadConfig(ArtifactPath("aos/events/aos_config.json"));
  return result;
}

// Tests just creating and destroying without running.
TEST(GlibMainLoopTest, CreateDestroy) {
  ShmEventLoop event_loop(Config());
  GlibMainLoop glib_main_loop(&event_loop);
}

// Tests just creating, running, and then destroying, without adding any
// events from the glib side.
TEST(GlibMainLoopTest, CreateRunDestroy) {
  ShmEventLoop event_loop(Config());
  GlibMainLoop glib_main_loop(&event_loop);
  bool ran = false;
  event_loop
      .AddTimer([&event_loop, &ran]() {
        event_loop.Exit();
        ran = true;
      })
      ->Schedule(event_loop.monotonic_now() + std::chrono::milliseconds(100));
  event_loop.Run();
  EXPECT_TRUE(ran);
}

// Tests just a single idle source.
TEST(GlibMainLoopTest, IdleSource) {
  ShmEventLoop event_loop(Config());
  GlibMainLoop glib_main_loop(&event_loop);
  int runs = 0;
  const auto callback =
      glib_main_loop.AddIdle([&event_loop, &runs]() -> gboolean {
        if (runs++ >= 100) {
          event_loop.Exit();
        }
        return true;
      });
  event_loop.Run();
  EXPECT_GT(runs, 100);
  // It can run a few extra times, but not too many.
  EXPECT_LT(runs, 110);
}

// Tests just a single timeout which calls exit on the ShmEventLoop side.
TEST(GlibMainLoopTest, TimeoutExitShm) {
  ShmEventLoop event_loop(Config());
  GlibMainLoop glib_main_loop(&event_loop);
  int runs = 0;
  const auto callback = glib_main_loop.AddTimeout(
      [&event_loop, &runs]() -> gboolean {
        if (runs++ >= 3) {
          event_loop.Exit();
        }
        return true;
      },
      50);
  const auto before = event_loop.monotonic_now();
  event_loop.Run();
  const auto after = event_loop.monotonic_now();
  EXPECT_EQ(runs, 4);
  // Verify it took at least this long, but don't bother putting an upper bound
  // because it can take arbitrarily long due to scheduling delays.
  EXPECT_GE(after - before, std::chrono::milliseconds(200));
}

// Tests just a single timeout which calls exit on the glib side.
TEST(GlibMainLoopTest, TimeoutExitGlib) {
  ShmEventLoop event_loop(Config());
  GlibMainLoop glib_main_loop(&event_loop);
  int runs = 0;
  const auto callback = glib_main_loop.AddTimeout(
      [&glib_main_loop, &runs]() -> gboolean {
        if (runs++ >= 3) {
          g_main_loop_quit(glib_main_loop.g_main_loop());
        }
        return true;
      },
      50);
  const auto before = event_loop.monotonic_now();
  event_loop.Run();
  const auto after = event_loop.monotonic_now();
  EXPECT_EQ(runs, 4);
  // Verify it took at least this long, but don't bother putting an upper bound
  // because it can take arbitrarily long due to scheduling delays.
  EXPECT_GE(after - before, std::chrono::milliseconds(200));
}

// Tests a single timeout which removes itself, and a ShmEventLoop timer to end
// the test.
TEST(GlibMainLoopTest, TimeoutRemoveSelf) {
  ShmEventLoop event_loop(Config());
  GlibMainLoop glib_main_loop(&event_loop);
  int runs = 0;
  const auto callback = glib_main_loop.AddTimeout(
      [&runs]() -> gboolean {
        ++runs;
        return false;
      },
      50);
  bool ran = false;
  event_loop
      .AddTimer([&event_loop, &ran]() {
        event_loop.Exit();
        ran = true;
      })
      ->Schedule(event_loop.monotonic_now() + std::chrono::milliseconds(100));
  event_loop.Run();
  EXPECT_TRUE(ran);
  EXPECT_EQ(runs, 1);
}

}  // namespace testing
}  // namespace aos
