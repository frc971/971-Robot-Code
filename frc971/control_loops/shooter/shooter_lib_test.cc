#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/control_loops/shooter/shooter.h"
#include "frc971/control_loops/shooter/unaugmented_shooter_motor_plant.h"
#include "frc971/constants.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

// Class which simulates the shooter and sends out queue messages containing the
// position.
class ShooterMotorSimulation {
 public:
  // Constructs a motor simulation.  initial_position is the inital angle of the
  // shooter, which will be treated as 0 by the encoder.
  ShooterMotorSimulation(double initial_position)
      : shooter_plant_(new StateFeedbackPlant<2, 1, 1>(MakeRawShooterPlant())),
        my_shooter_loop_(".frc971.control_loops.shooter",
                       0x1a7b7094, ".frc971.control_loops.shooter.goal",
                       ".frc971.control_loops.shooter.position",
                       ".frc971.control_loops.shooter.output",
                       ".frc971.control_loops.shooter.status") {

    printf("test");
    Reinitialize(initial_position);
  }

  // Resets the plant so that it starts at initial_position.
  void Reinitialize(double initial_position) {
    if(initial_position)
        return;
    else return;
  }

  // Returns the absolute angle of the shooter.
  double GetAbsolutePosition() const {
      return shooter_plant_->Y(0,0);
  }

  // Returns the adjusted angle of the shooter.
  double GetPosition() const {
    return shooter_plant_->Y(0, 0);
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double current_pos = GetPosition();

    ::aos::ScopedMessagePtr<control_loops::ShooterLoop::Position> position =
        my_shooter_loop_.position.MakeMessage();
    position->position = current_pos;

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    double abs_position = GetAbsolutePosition();
    if (abs_position >= constants::GetValues().shooter_lower_physical_limit &&
        abs_position <= constants::GetValues().shooter_upper_physical_limit) {
      //position->plunger_back_hall_effect = true;
    } else {
      //position->plunger_back_hall_effect = false;
    }

    // Only set calibration if it changed last cycle.  Calibration starts out
    // with a value of 0.
    if ((last_position_ <
             constants::GetValues().shooter_lower_physical_limit ||
         last_position_ >
             constants::GetValues().shooter_lower_physical_limit)/* &&
        position->hall_effect*/) {
      calibration_value_ =
          constants::GetValues().shooter_hall_effect_start_position -
          initial_position_;
    }

    position->back_calibration = calibration_value_;
    position.Send();
  }

  // Simulates the shooter moving for one timestep.
  void Simulate() {
    last_position_ = shooter_plant_->Y(0, 0);
    EXPECT_TRUE(my_shooter_loop_.output.FetchLatest());
    shooter_plant_->U << last_voltage_;
    shooter_plant_->Update();

    EXPECT_GE(constants::GetValues().shooter_upper_physical_limit,
              shooter_plant_->Y(0, 0));
    EXPECT_LE(constants::GetValues().shooter_lower_physical_limit,
              shooter_plant_->Y(0, 0));
    last_voltage_ = my_shooter_loop_.output->voltage;
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> shooter_plant_;
 private:
  ShooterLoop my_shooter_loop_;
  double initial_position_;
  double last_position_;
  double calibration_value_;
  double last_voltage_;
};

class ShooterTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ShooterLoop my_shooter_loop_;

  // Create a loop and simulation plant.
  ShooterMotor shooter_motor_;
  ShooterMotorSimulation shooter_motor_plant_;

  ShooterTest() : my_shooter_loop_(".frc971.control_loops.shooter",
                               0x1a7b7094, ".frc971.control_loops.shooter.goal",
                               ".frc971.control_loops.shooter.position",
                               ".frc971.control_loops.shooter.output",
                               ".frc971.control_loops.shooter.status"),
                shooter_motor_(&my_shooter_loop_),
                shooter_motor_plant_(0.5) {
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
    my_shooter_loop_.goal.FetchLatest();
    my_shooter_loop_.position.FetchLatest();
    EXPECT_NEAR(my_shooter_loop_.goal->goal,
                shooter_motor_plant_.GetAbsolutePosition(),
                1e-4);
  }

  virtual ~ShooterTest() {
    ::aos::robot_state.Clear();
  }
};

//TEST_F(ShooterTest, EmptyTest) {
//    EXPECT_TRUE(true);
//}
// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, ZerosCorrectly) {
  my_shooter_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 400; ++i) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that the shooter zeros correctly starting on the hall effect sensor.
//TEST_F(ShooterTest, ZerosStartingOn) {
//    printf("test");
//    EXPECT_TRUE(true);
//}

//// Tests that missing positions are correctly handled.
//TEST_F(ShooterTest, HandleMissingPosition) {
//}
//
//// Tests that losing the encoder for a second triggers a re-zero.
//TEST_F(ShooterTest, RezeroWithMissingPos) {
//}
//
//// Tests that disabling while zeroing sends the state machine into the
//// uninitialized state.
//TEST_F(ShooterTest, DisableGoesUninitialized) {
//}
//
//// Tests that the shooter can't get too far away from the zeroing position if the
//// zeroing position is saturating the goal.
//TEST_F(ShooterTest, NoWindupNegative) {
//}
//
//// Tests that the shooter can't get too far away from the zeroing position if the
//// zeroing position is saturating the goal.
//TEST_F(ShooterTest, NoWindupPositive) {
//}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
