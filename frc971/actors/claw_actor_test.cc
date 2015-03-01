#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/claw_action.q.h"
#include "frc971/actors/claw_actor.h"
#include "frc971/control_loops/claw/claw.q.h"

using ::aos::time::Time;

namespace frc971 {
namespace actors {
namespace testing {

class ClawActorTest : public ::testing::Test {
 protected:
  ClawActorTest() {
    frc971::actors::claw_action.goal.Clear();
    frc971::actors::claw_action.status.Clear();
    control_loops::claw_queue.status.Clear();
    control_loops::claw_queue.goal.Clear();
  }

  virtual ~ClawActorTest() {
    frc971::actors::claw_action.goal.Clear();
    frc971::actors::claw_action.status.Clear();
    control_loops::claw_queue.status.Clear();
    control_loops::claw_queue.goal.Clear();
  }

  // Bring up and down Core.
  ::aos::common::testing::GlobalCoreInstance my_core;
};

// Tests that it runs normally and exits when it should.
TEST_F(ClawActorTest, BasicTest) {
  ClawActor claw(&frc971::actors::claw_action);

  // Make some reasonable parameters.
  ClawParams params;
  params.angle = 0.5;
  params.max_velocity = 0.5;
  params.max_acceleration = 5.0;

  // Fake the status so that it thinks we're already there.
  control_loops::claw_queue.status.MakeWithBuilder()
      .angle(0.5)
      .goal_angle(0.5)
      .zeroed(true)
      .Send();

  // Now we run it and it should exit immediately.
  EXPECT_TRUE(claw.RunAction(params));

  // It shouldn't have sent us anywhere.
  ASSERT_TRUE(control_loops::claw_queue.goal.FetchLatest());
  EXPECT_EQ(params.angle, control_loops::claw_queue.goal->angle);
}

}  // namespace testing.
}  // namespace actors.
}  // namespace frc971.
