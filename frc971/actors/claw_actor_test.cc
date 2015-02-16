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
  params.claw_angle = 0.5;
  params.claw_max_velocity = 0.5;

  // Fake the status so that it thinks we're already there.
  control_loops::claw_queue.status.MakeWithBuilder()
      .angle(0.5)
      .zeroed(true)
      .Send();

  // Now we run it and it should exit immediately.
  EXPECT_TRUE(claw.RunAction(params));

  // It shouldn't have sent us anywhere.
  ASSERT_TRUE(control_loops::claw_queue.goal.FetchLatest());
  EXPECT_EQ(params.claw_angle, control_loops::claw_queue.goal->angle);
}

// Tests that it's outputting the goals we expect it to.
TEST_F(ClawActorTest, ValidGoals) {
  ClawActor claw(&frc971::actors::claw_action);

  // Make some reasonable parameters.
  ClawParams params;
  params.claw_angle = 0.5;
  params.claw_max_velocity = 0.5;

  // Set the starting parameters to what we want them to be.
  claw.delta_angle_ = 0.0;
  claw.claw_start_angle_ = 0.0;

  // Do the action iteration by iteration.
  double delta_goal = 0.0;
  while (!claw.Iterate(params)) {
    // Check that it sent a reasonable goal.
    control_loops::claw_queue.goal.FetchLatest();
    ASSERT_TRUE(control_loops::claw_queue.goal.get() != nullptr);

    delta_goal +=
        params.claw_max_velocity * ::aos::controls::kLoopFrequency.ToSeconds();
    EXPECT_EQ(delta_goal, control_loops::claw_queue.goal->angle);

    // Fake a status.
    control_loops::claw_queue.status.MakeWithBuilder().angle(delta_goal).Send();
  }

  delta_goal +=
      params.claw_max_velocity * ::aos::controls::kLoopFrequency.ToSeconds();
  EXPECT_NEAR(params.claw_angle, delta_goal, 0.01);
}

}  // namespace testing.
}  // namespace actors.
}  // namespace frc971.
