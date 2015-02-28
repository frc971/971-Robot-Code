#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/intake_action.q.h"
#include "frc971/actors/intake_actor.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/team_number_test_environment.h"

using ::aos::time::Time;

namespace frc971 {
namespace actors {
namespace testing {

class IntakeActionTest : public ::testing::Test {
 protected:
  IntakeActionTest() {
    frc971::actors::intake_action.goal.Clear();
    frc971::actors::intake_action.status.Clear();
    control_loops::fridge_queue.status.Clear();
    control_loops::fridge_queue.goal.Clear();
    control_loops::claw_queue.status.Clear();
    control_loops::claw_queue.goal.Clear();
  }

  virtual ~IntakeActionTest() {
    frc971::actors::intake_action.goal.Clear();
    frc971::actors::intake_action.status.Clear();
    control_loops::fridge_queue.status.Clear();
    control_loops::fridge_queue.goal.Clear();
    control_loops::claw_queue.status.Clear();
    control_loops::claw_queue.goal.Clear();
  }

  // Bring up and down Core.
  ::aos::common::testing::GlobalCoreInstance my_core;
};

// Tests that cancel stops not only the intake action, but also the underlying
// profile action.
TEST_F(IntakeActionTest, IntakeCancel) {
  IntakeActor intake(&frc971::actors::intake_action);

  frc971::actors::intake_action.goal.MakeWithBuilder().run(true).Send();

  // tell it the fridge and claw are zeroed
  control_loops::fridge_queue.status.MakeWithBuilder()
      .zeroed(true)
      .angle(0.0)
      .height(0.0)
      .Send();
  control_loops::claw_queue.status.MakeWithBuilder()
      .zeroed(true)
      .angle(0.0)
      .Send();

  // do the action and it will post to the goal queue
  intake.WaitForActionRequest();

  // the action has started, so now cancel it and it should cancel
  // the underlying profile
  frc971::actors::intake_action.goal.MakeWithBuilder().run(false).Send();

  // let the action start running, if we return from this call it has worked.
  const IntakeParams params = {true, true};
  intake.RunAction(params);

  SUCCEED();
}

}  // namespace testing
}  // namespace actors
}  // namespace frc971
