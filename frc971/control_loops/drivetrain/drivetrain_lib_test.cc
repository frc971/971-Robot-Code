#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_motor_plant.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {


// Class which simulates the drivetrain and sends out queue messages containing the
// position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  DrivetrainSimulation()
      : drivetrain_plant_(
            new StateFeedbackPlant<4, 2, 2>(MakeDrivetrainPlant())),
        my_drivetrain_loop_(".frc971.control_loops.drivetrain",
                       0x8a8dde77, ".frc971.control_loops.drivetrain.goal",
                       ".frc971.control_loops.drivetrain.position",
                       ".frc971.control_loops.drivetrain.output",
                       ".frc971.control_loops.drivetrain.status") {
    Reinitialize();
  }

  // Resets the plant.
  void Reinitialize() {
    drivetrain_plant_->X(0, 0) = 0.0;
    drivetrain_plant_->X(1, 0) = 0.0;
    drivetrain_plant_->Y = drivetrain_plant_->C * drivetrain_plant_->X;
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
  }

  // Returns the position of the drivetrain.
  double GetLeftPosition() const {
    return drivetrain_plant_->Y(0, 0);
  }
  double GetRightPosition() const {
    return drivetrain_plant_->Y(1, 0);
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double left_encoder = GetLeftPosition();
    const double right_encoder = GetRightPosition();

    ::aos::ScopedMessagePtr<control_loops::Drivetrain::Position> position =
        my_drivetrain_loop_.position.MakeMessage();
    position->left_encoder = left_encoder;
    position->right_encoder = right_encoder;
    position.Send();
  }

  // Simulates the drivetrain moving for one timestep.
  void Simulate() {
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
    EXPECT_TRUE(my_drivetrain_loop_.output.FetchLatest());
    drivetrain_plant_->U << my_drivetrain_loop_.output->left_voltage,
                            my_drivetrain_loop_.output->right_voltage;
    drivetrain_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> drivetrain_plant_;
 private:
  Drivetrain my_drivetrain_loop_;
  double last_left_position_;
  double last_right_position_;
};

class DrivetrainTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  Drivetrain my_drivetrain_loop_;

  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;

  DrivetrainTest() : my_drivetrain_loop_(".frc971.control_loops.drivetrain",
                               0x8a8dde77,
                               ".frc971.control_loops.drivetrain.goal",
                               ".frc971.control_loops.drivetrain.position",
                               ".frc971.control_loops.drivetrain.output",
                               ".frc971.control_loops.drivetrain.status"),
                drivetrain_motor_(&my_drivetrain_loop_),
                drivetrain_motor_plant_() {
    // Flush the robot state queue so we can use clean shared memory for this
    // test.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
  }

  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled)
                                        .autonomous(false)
                                        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

  void VerifyNearGoal() {
    my_drivetrain_loop_.goal.FetchLatest();
    my_drivetrain_loop_.position.FetchLatest();
    EXPECT_NEAR(my_drivetrain_loop_.goal->left_goal,
                drivetrain_motor_plant_.GetLeftPosition(),
                1e-2);
    EXPECT_NEAR(my_drivetrain_loop_.goal->right_goal,
                drivetrain_motor_plant_.GetRightPosition(),
                1e-2);
  }

  virtual ~DrivetrainTest() {
    ::aos::robot_state.Clear();
  }
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  my_drivetrain_loop_.goal.MakeWithBuilder().control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0).Send();
  for (int i = 0; i < 200; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
