#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/wrist_motor.q.h"
#include "frc971/control_loops/wrist.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

class WristTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  WristLoop my_wrist_loop_;

  WristMotor wrist_motor_;
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> wrist_plant_;

  WristTest() : my_wrist_loop_(".frc971.control_loops.wrist",
                               0x1a7b7094, ".frc971.control_loops.wrist.goal",
                               ".frc971.control_loops.wrist.position",
                               ".frc971.control_loops.wrist.output",
                               ".frc971.control_loops.wrist.status"),
                wrist_motor_(&my_wrist_loop_),
                wrist_plant_(new StateFeedbackPlant<2, 1, 1>(MakeWristPlant())) {
    // Flush the robot state queue so we can recreate shared memory for this
    // test.
    ::aos::robot_state.Clear();
    ::aos::robot_state.MakeWithBuilder().enabled(true)
                                        .autonomous(false)
                                        .team_id(971).Send();
  }
};


// Tests that we can send a message to another thread and it blocking receives
// it at the correct time.
TEST_F(WristTest, Stable) {
  my_wrist_loop_.goal.MakeWithBuilder().goal(0.0).Send();
  for (int i = 0; i < 1000; ++i) {
    my_wrist_loop_.position.MakeWithBuilder()
        .pos(wrist_plant_->Y(0, 0))
        .hall_effect(false)
        .calibration(0.0).Send();

    wrist_motor_.Iterate();

    EXPECT_TRUE(my_wrist_loop_.output.FetchLatest());
    wrist_plant_->U << my_wrist_loop_.output->voltage;
    wrist_plant_->Update();
  }
  my_wrist_loop_.position.FetchLatest();
  EXPECT_FLOAT_EQ(0.0, my_wrist_loop_.position->pos);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
