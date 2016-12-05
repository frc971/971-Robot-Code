#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/testing/test_shm.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/score_action.q.h"
#include "y2015/actors/score_actor.h"
#include "y2015/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/team_number_test_environment.h"

using ::aos::time::Time;

namespace y2015 {
namespace actors {
namespace testing {

using ::y2015::control_loops::fridge::fridge_queue;

class ScoreActionTest : public ::testing::Test {
 protected:
  ScoreActionTest() {
    y2015::actors::score_action.goal.Clear();
    y2015::actors::score_action.status.Clear();
    fridge_queue.status.Clear();
    fridge_queue.goal.Clear();
  }

  virtual ~ScoreActionTest() {
    y2015::actors::score_action.goal.Clear();
    y2015::actors::score_action.status.Clear();
    fridge_queue.status.Clear();
    fridge_queue.goal.Clear();
  }

  // Bring up and down Core.
  ::aos::testing::TestSharedMemory my_shm_;
};

// Tests that cancel stops not only the score action, but also the underlying
// profile action.
TEST_F(ScoreActionTest, PlaceTheStackCancel) {
  ScoreActor score(&y2015::actors::score_action);

  y2015::actors::score_action.goal.MakeWithBuilder().run(true).Send();

  // Tell it the fridge is zeroed.
  ASSERT_TRUE(fridge_queue.status.MakeWithBuilder()
                  .zeroed(true)
                  .angle(0.0)
                  .height(0.0)
                  .Send());

  // do the action and it will post to the goal queue
  score.WaitForActionRequest();

  // the action has started, so now cancel it and it should cancel
  // the underlying profile
  y2015::actors::score_action.goal.MakeWithBuilder().run(false).Send();

  // let the action start running, if we return from this call it has worked.
  const ScoreParams params = {true, true, 0.14, 0.13, -0.7, -0.7, -0.10, -0.5, 0.1};
  score.RunAction(params);

  SUCCEED();
}

// Tests that cancel stops not only the score action, but also the underlying
// profile action.
TEST_F(ScoreActionTest, MoveStackIntoPositionCancel) {
  ScoreActor score(&y2015::actors::score_action);

  y2015::actors::score_action.goal.MakeWithBuilder().run(true).Send();

  // Tell it the fridge is zeroed.
  ASSERT_TRUE(fridge_queue.status.MakeWithBuilder()
                  .zeroed(true)
                  .angle(0.0)
                  .height(0.0)
                  .Send());

  // do the action and it will post to the goal queue
  score.WaitForActionRequest();

  // the action has started, so now cancel it and it should cancel
  // the underlying profile
  y2015::actors::score_action.goal.MakeWithBuilder().run(false).Send();

  // let the action start running, if we return from this call it has worked.
  const ScoreParams params = {false, true, 0.14, 0.13, -0.7, -0.7, -0.10, -0.5, 0.1};
  score.RunAction(params);

  SUCCEED();
}

}  // namespace testing
}  // namespace actors
}  // namespace y2015
