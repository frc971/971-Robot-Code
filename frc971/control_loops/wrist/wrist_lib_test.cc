#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/wrist/wrist.h"
#include "frc971/constants.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {


// Class which simulates the wrist and sends out queue messages containing the
// position.
class WristMotorSimulation {
 public:
  // Constructs a motor simulation.  initial_position is the inital angle of the
  // wrist, which will be treated as 0 by the encoder.
  WristMotorSimulation(double initial_position)
      : wrist_plant_(new StateFeedbackPlant<2, 1, 1>(MakeRawWristPlant())),
        my_wrist_loop_(".frc971.control_loops.wrist",
                       0x1a7b7094, ".frc971.control_loops.wrist.goal",
                       ".frc971.control_loops.wrist.position",
                       ".frc971.control_loops.wrist.output",
                       ".frc971.control_loops.wrist.status") {
    Reinitialize(initial_position);
  }

  // Resets the plant so that it starts at initial_position.
  void Reinitialize(double initial_position) {
    initial_position_ = initial_position;
    wrist_plant_->X(0, 0) = initial_position_;
    wrist_plant_->X(1, 0) = 0.0;
    wrist_plant_->Y = wrist_plant_->C() * wrist_plant_->X;
    last_position_ = wrist_plant_->Y(0, 0);
    calibration_value_ = 0.0;
    last_voltage_ = 0.0;
  }

  // Returns the absolute angle of the wrist.
  double GetAbsolutePosition() const {
    return wrist_plant_->Y(0, 0);
  }

  // Returns the adjusted angle of the wrist.
  double GetPosition() const {
    return GetAbsolutePosition() - initial_position_;
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double angle = GetPosition();

    double wrist_hall_effect_start_angle;
    ASSERT_TRUE(constants::wrist_hall_effect_start_angle(
                    &wrist_hall_effect_start_angle));
    double wrist_hall_effect_stop_angle;
    ASSERT_TRUE(constants::wrist_hall_effect_stop_angle(
                    &wrist_hall_effect_stop_angle));

    ::aos::ScopedMessagePtr<control_loops::WristLoop::Position> position =
        my_wrist_loop_.position.MakeMessage();
    position->pos = angle;

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    double abs_position = GetAbsolutePosition();
    if (abs_position >= wrist_hall_effect_start_angle &&
        abs_position <= wrist_hall_effect_stop_angle) {
      position->hall_effect = true;
    } else {
      position->hall_effect = false;
    }

    // Only set calibration if it changed last cycle.  Calibration starts out
    // with a value of 0.
    if ((last_position_ < wrist_hall_effect_start_angle ||
         last_position_ > wrist_hall_effect_stop_angle) &&
         position->hall_effect) {
      calibration_value_ =
          wrist_hall_effect_start_angle - initial_position_;
    }

    position->calibration = calibration_value_;
    position.Send();
  }

  // Simulates the wrist moving for one timestep.
  void Simulate() {
    last_position_ = wrist_plant_->Y(0, 0);
    EXPECT_TRUE(my_wrist_loop_.output.FetchLatest());
    wrist_plant_->U << last_voltage_;
    wrist_plant_->Update();

    // Assert that we are in the right physical range.
    double wrist_upper_physical_limit;
    ASSERT_TRUE(constants::wrist_upper_physical_limit(
                    &wrist_upper_physical_limit));
    double wrist_lower_physical_limit;
    ASSERT_TRUE(constants::wrist_lower_physical_limit(
                    &wrist_lower_physical_limit));

    EXPECT_GE(wrist_upper_physical_limit,
              wrist_plant_->Y(0, 0));
    EXPECT_LE(wrist_lower_physical_limit,
              wrist_plant_->Y(0, 0));
    last_voltage_ = my_wrist_loop_.output->voltage;
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> wrist_plant_;
 private:
  WristLoop my_wrist_loop_;
  double initial_position_;
  double last_position_;
  double calibration_value_;
  double last_voltage_;
};

class WristTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  WristLoop my_wrist_loop_;

  // Create a loop and simulation plant.
  WristMotor wrist_motor_;
  WristMotorSimulation wrist_motor_plant_;

  WristTest() : my_wrist_loop_(".frc971.control_loops.wrist",
                               0x1a7b7094, ".frc971.control_loops.wrist.goal",
                               ".frc971.control_loops.wrist.position",
                               ".frc971.control_loops.wrist.output",
                               ".frc971.control_loops.wrist.status"),
                wrist_motor_(&my_wrist_loop_),
                wrist_motor_plant_(0.5) {
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
    my_wrist_loop_.goal.FetchLatest();
    my_wrist_loop_.position.FetchLatest();
    EXPECT_NEAR(my_wrist_loop_.goal->goal,
                wrist_motor_plant_.GetAbsolutePosition(),
                1e-4);
  }

  virtual ~WristTest() {
    ::aos::robot_state.Clear();
  }
};

// Tests that the wrist zeros correctly and goes to a position.
TEST_F(WristTest, ZerosCorrectly) {
  my_wrist_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 400; ++i) {
    wrist_motor_plant_.SendPositionMessage();
    wrist_motor_.Iterate();
    wrist_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that the wrist zeros correctly starting on the hall effect sensor.
TEST_F(WristTest, ZerosStartingOn) {
  wrist_motor_plant_.Reinitialize(90 * M_PI / 180.0);

  my_wrist_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 500; ++i) {
    wrist_motor_plant_.SendPositionMessage();
    wrist_motor_.Iterate();
    wrist_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that missing positions are correctly handled.
TEST_F(WristTest, HandleMissingPosition) {
  my_wrist_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 400; ++i) {
    if (i % 23) {
      wrist_motor_plant_.SendPositionMessage();
    }
    wrist_motor_.Iterate();
    wrist_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that loosing the encoder for a second triggers a re-zero.
TEST_F(WristTest, RezeroWithMissingPos) {
  my_wrist_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 800; ++i) {
    // After 3 seconds, simulate the encoder going missing.
    // This should trigger a re-zero.  To make sure it works, change the goal as
    // well.
    if (i < 300 || i > 400) {
      wrist_motor_plant_.SendPositionMessage();
    } else {
      if (i > 310) {
        // Should be re-zeroing now.
        EXPECT_TRUE(wrist_motor_.is_uninitialized());
      }
      my_wrist_loop_.goal.MakeWithBuilder().goal(0.2).Send();
    }
    if (i == 410) {
      EXPECT_TRUE(wrist_motor_.is_zeroing());
    }

    wrist_motor_.Iterate();
    wrist_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that disabling while zeroing sends the state machine into the
// uninitialized state.
TEST_F(WristTest, DisableGoesUninitialized) {
  my_wrist_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 800; ++i) {
    wrist_motor_plant_.SendPositionMessage();
    // After 0.5 seconds, disable the robot.
    if (i > 50 && i < 200) {
      SendDSPacket(false);
      if (i > 100) {
        // Give the loop a couple cycled to get the message and then verify that
        // it is in the correct state.
        EXPECT_TRUE(wrist_motor_.is_uninitialized());
        // When disabled, we should be applying 0 power.
        EXPECT_TRUE(my_wrist_loop_.output.FetchLatest());
        EXPECT_EQ(0, my_wrist_loop_.output->voltage);
      }
    } else {
      SendDSPacket(true);
    }
    if (i == 202) {
      // Verify that we are zeroing after the bot gets enabled again.
      EXPECT_TRUE(wrist_motor_.is_zeroing());
    }

    wrist_motor_.Iterate();
    wrist_motor_plant_.Simulate();
  }
  VerifyNearGoal();
}

// Tests that the wrist can't get too far away from the zeroing position if the
// zeroing position is saturating the goal.
TEST_F(WristTest, NoWindupNegative) {
  int capped_count = 0;
  double saved_zeroing_position = 0;
  my_wrist_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 500; ++i) {
    wrist_motor_plant_.SendPositionMessage();
    if (i == 50) {
      EXPECT_TRUE(wrist_motor_.is_zeroing());
      // Move the zeroing position far away and verify that it gets moved back.
      saved_zeroing_position = wrist_motor_.zeroed_joint_.zeroing_position_;
      wrist_motor_.zeroed_joint_.zeroing_position_ = -100.0;
    } else if (i == 51) {
      EXPECT_TRUE(wrist_motor_.is_zeroing());
      EXPECT_NEAR(saved_zeroing_position,
                  wrist_motor_.zeroed_joint_.zeroing_position_, 0.4);
    }
    if (!wrist_motor_.is_ready()) {
      if (wrist_motor_.capped_goal()) {
        ++capped_count;
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(wrist_motor_.zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(wrist_motor_.zeroed_joint_.U_uncapped()));
      }
    }

    wrist_motor_.Iterate();
    wrist_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
  EXPECT_GT(3, capped_count);
}

// Tests that the wrist can't get too far away from the zeroing position if the
// zeroing position is saturating the goal.
TEST_F(WristTest, NoWindupPositive) {
  int capped_count = 0;
  double saved_zeroing_position = 0;
  my_wrist_loop_.goal.MakeWithBuilder().goal(0.1).Send();
  for (int i = 0; i < 500; ++i) {
    wrist_motor_plant_.SendPositionMessage();
    if (i == 50) {
      EXPECT_TRUE(wrist_motor_.is_zeroing());
      // Move the zeroing position far away and verify that it gets moved back.
      saved_zeroing_position = wrist_motor_.zeroed_joint_.zeroing_position_;
      wrist_motor_.zeroed_joint_.zeroing_position_ = 100.0;
    } else {
      if (i == 51) {
        EXPECT_TRUE(wrist_motor_.is_zeroing());
        EXPECT_NEAR(saved_zeroing_position, wrist_motor_.zeroed_joint_.zeroing_position_, 0.4);
      }
    }
    if (!wrist_motor_.is_ready()) {
      if (wrist_motor_.capped_goal()) {
        ++capped_count;
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        EXPECT_LT(5, ::std::abs(wrist_motor_.zeroed_joint_.U_uncapped()));
      } else {
        EXPECT_GT(5, ::std::abs(wrist_motor_.zeroed_joint_.U_uncapped()));
      }
    }

    wrist_motor_.Iterate();
    wrist_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
  EXPECT_GT(3, capped_count);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
