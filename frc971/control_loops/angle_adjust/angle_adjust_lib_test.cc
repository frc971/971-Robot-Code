#include <unistd.h>

#include <memory>
#include <array>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust.h"
#include "frc971/control_loops/angle_adjust/unaugmented_angle_adjust_motor_plant.h"
#include "frc971/constants.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {


// Class which simulates the angle_adjust and
// sends out queue messages containing the position.
class AngleAdjustMotorSimulation {
 public:
  // Constructs a motor simulation.  initial_position is the inital angle of the
  // angle_adjust, which will be treated as 0 by the encoder.
  explicit AngleAdjustMotorSimulation(double initial_position)
      : angle_adjust_plant_(
          new StateFeedbackPlant<2, 1, 1>(MakeRawAngleAdjustPlant())),
        my_angle_adjust_loop_(".frc971.control_loops.angle_adjust",
                       0x65c7ef53, ".frc971.control_loops.angle_adjust.goal",
                       ".frc971.control_loops.angle_adjust.position",
                       ".frc971.control_loops.angle_adjust.output",
                       ".frc971.control_loops.angle_adjust.status") {
    Reinitialize(initial_position);
  }

  // Resets the plant so that it starts at initial_position.
  void Reinitialize(double initial_position) {
    initial_position_ = initial_position;
    angle_adjust_plant_->X(0, 0) = initial_position_;
    angle_adjust_plant_->X(1, 0) = 0.0;
    angle_adjust_plant_->Y = angle_adjust_plant_->C() * angle_adjust_plant_->X;
    last_position_ = angle_adjust_plant_->Y(0, 0);
    last_voltage_ = 0.0;
    calibration_value_[0] = 0.0;
    calibration_value_[1] = 0.0;
  }

  // Returns the absolute angle of the angle_adjust.
  double GetAbsolutePosition() const {
    return angle_adjust_plant_->Y(0, 0);
  }

  // Returns the adjusted angle of the angle_adjust.
  double GetPosition() const {
    return GetAbsolutePosition() - initial_position_;
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double angle = GetPosition();

    ::aos::ScopedMessagePtr<control_loops::AngleAdjustLoop::Position> position =
        my_angle_adjust_loop_.position.MakeMessage();
    position->angle = angle;

    const double (&hall_effect_start_angle)[2] =
        constants::GetValues().angle_adjust_hall_effect_start_angle;
    const double (&hall_effect_stop_angle)[2] =
        constants::GetValues().angle_adjust_hall_effect_stop_angle;

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    double abs_position = GetAbsolutePosition();
    if (abs_position <= hall_effect_start_angle[0] &&
        abs_position >= hall_effect_stop_angle[0]) {
      position->bottom_hall_effect = true;
    } else {
      position->bottom_hall_effect = false;
    }
    if (abs_position <= hall_effect_start_angle[1] &&
        abs_position >= hall_effect_stop_angle[1]) {
      position->middle_hall_effect = true;
    } else {
      position->middle_hall_effect = false;
    }

    // Only set calibration if it changed last cycle.  Calibration starts out
    // with a value of 0.
    // TODO(aschuh): This won't deal with both edges correctly.
    if ((last_position_ < hall_effect_start_angle[0] ||
         last_position_ > hall_effect_stop_angle[0]) &&
         (position->bottom_hall_effect)) {
      calibration_value_[0] = hall_effect_start_angle[0] - initial_position_;
    }
    if ((last_position_ < hall_effect_start_angle[1] ||
         last_position_ > hall_effect_stop_angle[1]) &&
         (position->middle_hall_effect)) {
      calibration_value_[1] = hall_effect_start_angle[1] - initial_position_;
    }

    position->bottom_calibration = calibration_value_[0];
    position->middle_calibration = calibration_value_[1];
    position.Send();
  }

  // Simulates the angle_adjust moving for one timestep.
  void Simulate() {
    last_position_ = angle_adjust_plant_->Y(0, 0);
    EXPECT_TRUE(my_angle_adjust_loop_.output.FetchLatest());
    angle_adjust_plant_->U << last_voltage_;
    angle_adjust_plant_->Update();

    EXPECT_GE(constants::GetValues().angle_adjust_upper_physical_limit,
              angle_adjust_plant_->Y(0, 0));
    EXPECT_LE(constants::GetValues().angle_adjust_lower_physical_limit,
              angle_adjust_plant_->Y(0, 0));
    last_voltage_ = my_angle_adjust_loop_.output->voltage;
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> angle_adjust_plant_;

 private:
  AngleAdjustLoop my_angle_adjust_loop_;
  double initial_position_;
  double last_position_;
  double last_voltage_;
  double calibration_value_[2];
};

class AngleAdjustTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  AngleAdjustLoop my_angle_adjust_loop_;

  // Create a loop and simulation plant.
  AngleAdjustMotor angle_adjust_motor_;
  AngleAdjustMotorSimulation angle_adjust_motor_plant_;

  AngleAdjustTest() :
    my_angle_adjust_loop_(".frc971.control_loops.angle_adjust",
                          0x65c7ef53, ".frc971.control_loops.angle_adjust.goal",
                          ".frc971.control_loops.angle_adjust.position",
                          ".frc971.control_loops.angle_adjust.output",
                          ".frc971.control_loops.angle_adjust.status"),
    angle_adjust_motor_(&my_angle_adjust_loop_),
    angle_adjust_motor_plant_(0.75) {
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
    my_angle_adjust_loop_.goal.FetchLatest();
    my_angle_adjust_loop_.position.FetchLatest();
    EXPECT_NEAR(my_angle_adjust_loop_.goal->goal,
                angle_adjust_motor_plant_.GetAbsolutePosition(),
                1e-4);
  }

  virtual ~AngleAdjustTest() {
    ::aos::robot_state.Clear();
  }
};

// Tests that the angle_adjust zeros correctly and goes to a position.
TEST_F(AngleAdjustTest, ZerosCorrectly) {
  my_angle_adjust_loop_.goal.MakeWithBuilder().goal(0.4).Send();
  for (int i = 0; i < 400; ++i) {
    angle_adjust_motor_plant_.SendPositionMessage();
    angle_adjust_motor_.Iterate();
    angle_adjust_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that the angle_adjust zeros correctly starting on the sensor.
TEST_F(AngleAdjustTest, ZerosStartingOn) {
  angle_adjust_motor_plant_.Reinitialize(0.30);
  my_angle_adjust_loop_.goal.MakeWithBuilder().goal(0.4).Send();
  for (int i = 0; i < 500; ++i) {
    angle_adjust_motor_plant_.SendPositionMessage();
    angle_adjust_motor_.Iterate();
    angle_adjust_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that missing positions are correctly handled.
TEST_F(AngleAdjustTest, HandleMissingPosition) {
  my_angle_adjust_loop_.goal.MakeWithBuilder().goal(0.4).Send();
  for (int i = 0; i < 400; ++i) {
    if (i % 23) {
      angle_adjust_motor_plant_.SendPositionMessage();
    }
    angle_adjust_motor_.Iterate();
    angle_adjust_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that losing the encoder for 11 seconds triggers a re-zero.
TEST_F(AngleAdjustTest, RezeroWithMissingPos) {
  my_angle_adjust_loop_.goal.MakeWithBuilder().goal(0.4).Send();
  for (int i = 0; i < 1800; ++i) {
    // After 3 seconds, simulate the encoder going missing.
    // This should trigger a re-zero.  To make sure it works, change the goal as
    // well.
    if (i < 300 || i > 1400) {
      angle_adjust_motor_plant_.SendPositionMessage();
    } else {
      if (i > 1310) {
        // Should be re-zeroing now.
        EXPECT_TRUE(angle_adjust_motor_.is_uninitialized());
      }
      my_angle_adjust_loop_.goal.MakeWithBuilder().goal(0.5).Send();
    }
    if (i == 1430) {
      EXPECT_TRUE(angle_adjust_motor_.is_zeroing() ||
                  angle_adjust_motor_.is_moving_off());
    }

    angle_adjust_motor_.Iterate();
    angle_adjust_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that disabling while zeroing sends the state machine into the
// uninitialized state.
TEST_F(AngleAdjustTest, DisableGoesUninitialized) {
  my_angle_adjust_loop_.goal.MakeWithBuilder().goal(0.4).Send();
  for (int i = 0; i < 800; ++i) {
    angle_adjust_motor_plant_.SendPositionMessage();
    // After 0.5 seconds, disable the robot.
    if (i > 50 && i < 200) {
      SendDSPacket(false);
      if (i > 100) {
        // Give the loop a couple cycled to get the message and then verify that
        // it is in the correct state.
        EXPECT_TRUE(angle_adjust_motor_.is_uninitialized());
      }
    } else {
      SendDSPacket(true);
    }
    if (i == 202) {
      // Verify that we are zeroing after the bot gets enabled again.
      EXPECT_TRUE(angle_adjust_motor_.is_zeroing());
    }

    angle_adjust_motor_.Iterate();
    angle_adjust_motor_plant_.Simulate();
  }
  VerifyNearGoal();
}

/*
// TODO(aschuh): Enable these tests if we install a second hall effect sensor.
// Tests that the angle_adjust zeros correctly from above the second sensor.
TEST_F(AngleAdjustTest, ZerosCorrectlyAboveSecond) {
  angle_adjust_motor_plant_.Reinitialize(1.75);
  my_angle_adjust_loop_.goal.MakeWithBuilder().goal(1.0).Send();
  for (int i = 0; i < 400; ++i) {
    angle_adjust_motor_plant_.SendPositionMessage();
    angle_adjust_motor_.Iterate();
    angle_adjust_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that the angle_adjust zeros correctly starting on
// the second hall effect sensor.
TEST_F(AngleAdjustTest, ZerosStartingOnSecond) {
  angle_adjust_motor_plant_.Reinitialize(1.25);
  my_angle_adjust_loop_.goal.MakeWithBuilder().goal(1.0).Send();
  for (int i = 0; i < 500; ++i) {
    angle_adjust_motor_plant_.SendPositionMessage();
    angle_adjust_motor_.Iterate();
    angle_adjust_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}
*/

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
