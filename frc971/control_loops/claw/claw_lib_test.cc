#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw.h"
#include "frc971/constants.h"
#include "frc971/control_loops/claw/unaugmented_top_claw_motor_plant.h"
#include "frc971/control_loops/claw/unaugmented_bottom_claw_motor_plant.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

typedef enum {
	TOP_CLAW = 0,
	BOTTOM_CLAW,

	CLAW_COUNT
} ClawType;

// Class which simulates the wrist and sends out queue messages containing the
// position.
class ClawMotorSimulation {
 public:
  // Constructs a motor simulation.  initial_position is the inital angle of the
  // wrist, which will be treated as 0 by the encoder.
  ClawMotorSimulation(double initial_top_position,
                      double initial_bottom_position)
      : top_claw_plant_(new StateFeedbackPlant<2, 1, 1>(MakeRawTopClawPlant())),
        bottom_claw_plant_(
            new StateFeedbackPlant<2, 1, 1>(MakeRawBottomClawPlant())),
        claw_queue_group(".frc971.control_loops.claw", 0x1a7b7094,
                         ".frc971.control_loops.claw.goal",
                         ".frc971.control_loops.claw.position",
                         ".frc971.control_loops.claw.output",
                         ".frc971.control_loops.claw.status") {
    Reinitialize(TOP_CLAW, initial_top_position);
    Reinitialize(BOTTOM_CLAW, initial_bottom_position);
  }

  // Resets the plant so that it starts at initial_position.
  void Reinitialize(ClawType type, double initial_position) {
    StateFeedbackPlant<2, 1, 1>* plant;
    if (type == TOP_CLAW) {
      plant = top_claw_plant_.get();
    } else {
      plant = bottom_claw_plant_.get();
    }
    initial_position_[type] = initial_position;
    plant->X(0, 0) = initial_position_[type];
    plant->X(1, 0) = 0.0;
    plant->Y = plant->C() * plant->X;
    last_position_[type] = plant->Y(0, 0);
    calibration_value_[type] = 0.0;
    last_voltage_[type] = 0.0;
  }

  // Returns the absolute angle of the wrist.
  double GetAbsolutePosition(ClawType type) const {
    if (type == TOP_CLAW) {
      return top_claw_plant_->Y(0, 0);
    } else {
      return bottom_claw_plant_->Y(0, 0);
    }
  }

  // Returns the adjusted angle of the wrist.
  double GetPosition(ClawType type) const {
    return GetAbsolutePosition(type) - initial_position_[type];
  }

  // Makes sure pos is inside range (inclusive)
  bool CheckRange(double pos, double low_limit, double hi_limit) {
    return (pos >= low_limit && pos <= hi_limit);
  }

  double CheckCalibration(ClawType type, bool hall_effect, double start_angle,
                          double stop_angle) {
    if ((last_position_[type] < start_angle ||
         last_position_[type] > stop_angle) &&
        hall_effect) {
      calibration_value_[type] = start_angle - initial_position_[type];
    }
    return calibration_value_[type];
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::ClawGroup::Position> position =
        claw_queue_group.position.MakeMessage();
    position->top_position = GetPosition(TOP_CLAW);
    position->bottom_position = GetPosition(BOTTOM_CLAW);

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    double pos[2] = {GetAbsolutePosition(TOP_CLAW),
                     GetAbsolutePosition(BOTTOM_CLAW)};
    const frc971::constants::Values& v = constants::GetValues();
    position->top_front_hall_effect =
        CheckRange(pos[TOP_CLAW], v.claw_front_heffect_start_angle,
                   v.claw_front_heffect_stop_angle);
    position->top_calibration_hall_effect =
        CheckRange(pos[TOP_CLAW], v.claw_calib_heffect_start_angle,
                   v.claw_calib_heffect_stop_angle);
    position->top_back_hall_effect =
        CheckRange(pos[TOP_CLAW], v.claw_back_heffect_start_angle,
                   v.claw_back_heffect_stop_angle);
    position->bottom_front_hall_effect =
        CheckRange(pos[TOP_CLAW], v.claw_front_heffect_start_angle,
                   v.claw_front_heffect_stop_angle);
    position->bottom_calibration_hall_effect =
        CheckRange(pos[TOP_CLAW], v.claw_calib_heffect_start_angle,
                   v.claw_calib_heffect_stop_angle);
    position->bottom_back_hall_effect =
        CheckRange(pos[TOP_CLAW], v.claw_back_heffect_start_angle,
                   v.claw_back_heffect_stop_angle);

    // Only set calibration if it changed last cycle.  Calibration starts out
    // with a value of 0.
    position->top_calibration = CheckCalibration(
        TOP_CLAW, position->top_calibration_hall_effect,
        v.claw_calib_heffect_start_angle, v.claw_calib_heffect_stop_angle);
    position->bottom_calibration = CheckCalibration(
        BOTTOM_CLAW, position->bottom_calibration_hall_effect,
        v.claw_calib_heffect_start_angle, v.claw_calib_heffect_stop_angle);
    position.Send();
  }

  // Simulates the claw moving for one timestep.
  void Simulate() {
    const frc971::constants::Values& v = constants::GetValues();
    EXPECT_TRUE(claw_queue_group.output.FetchLatest());
    Simulate(TOP_CLAW, top_claw_plant_.get(), v.claw_upper_limit,
             v.claw_lower_limit, claw_queue_group.output->top_claw_voltage);
    Simulate(BOTTOM_CLAW, bottom_claw_plant_.get(), v.claw_upper_limit,
             v.claw_lower_limit, claw_queue_group.output->bottom_claw_voltage);
  }

  void Simulate(ClawType type, StateFeedbackPlant<2, 1, 1>* plant,
                double upper_limit, double lower_limit, double nl_voltage) {
    last_position_[type] = plant->Y(0, 0);
    plant->U << last_voltage_[type];
    plant->Update();

    // check top claw inside limits
    EXPECT_GE(upper_limit, plant->Y(0, 0));
    EXPECT_LE(lower_limit, plant->Y(0, 0));
    // check bottom claw inside limits
    EXPECT_GE(upper_limit, plant->Y(0, 0));
    EXPECT_LE(lower_limit, plant->Y(0, 0));
    last_voltage_[type] = nl_voltage;
  }

  // Top of the claw, the one with rollers
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> top_claw_plant_;
  // Bottom of the claw, the one with tusks
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> bottom_claw_plant_;
 private:
  ClawGroup claw_queue_group;
  double initial_position_[CLAW_COUNT];
  double last_position_[CLAW_COUNT];
  double calibration_value_[CLAW_COUNT];
  double last_voltage_[CLAW_COUNT];
};


class ClawTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ClawGroup claw_queue_group;

  // Create a loop and simulation plant.
  ClawMotor claw_motor_;
  ClawMotorSimulation claw_motor_plant_;

  // Minimum amount of acceptable seperation between the top and bottom of the
  // claw.
  double min_seperation_;

  ClawTest()
      : claw_queue_group(".frc971.control_loops.wrist", 0x1a7b7094,
                         ".frc971.control_loops.wrist.goal",
                         ".frc971.control_loops.wrist.position",
                         ".frc971.control_loops.wrist.output",
                         ".frc971.control_loops.wrist.status"),
        claw_motor_(&claw_queue_group),
        claw_motor_plant_(0.5, 1.0),
        min_seperation_(constants::GetValues().claw_min_seperation) {
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
    claw_queue_group.goal.FetchLatest();
    claw_queue_group.position.FetchLatest();
    double bottom = claw_motor_plant_.GetAbsolutePosition(BOTTOM_CLAW);
    double seperation =
        claw_motor_plant_.GetAbsolutePosition(TOP_CLAW) - bottom;
    EXPECT_NEAR(claw_queue_group.goal->bottom_angle, bottom, 1e-4);
    EXPECT_NEAR(claw_queue_group.goal->seperation_angle, seperation, 1e-4);
    EXPECT_TRUE(min_seperation_ <= seperation);
  }

  virtual ~ClawTest() {
    ::aos::robot_state.Clear();
  }
};

// Tests that the wrist zeros correctly and goes to a position.
TEST_F(ClawTest, ZerosCorrectly) {
  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 400; ++i) {
    claw_motor_plant_.SendPositionMessage();
    claw_motor_.Iterate();
    bottom_claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that the wrist zeros correctly starting on the hall effect sensor.
TEST_F(ClawTest, ZerosStartingOn) {
  claw_motor_plant_.Reinitialize(TOP_CLAW, 90 * M_PI / 180.0);
  claw_motor_plant_.Reinitialize(BOTTOM_CLAW, 100 * M_PI / 180.0);

  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 500; ++i) {
    claw_motor_plant_.SendPositionMessage();
    claw_motor_.Iterate();
    bottom_claw_motor_.Iterate();
    claw_motor_plant_.Simulate();

    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that missing positions are correctly handled.
TEST_F(ClawTest, HandleMissingPosition) {
  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 400; ++i) {
    if (i % 23) {
      claw_motor_plant_.SendPositionMessage();
    }
    claw_motor_.Iterate();
    bottom_claw_motor_.Iterate();
    claw_motor_plant_.Simulate();

    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that loosing the encoder for a second triggers a re-zero.
TEST_F(ClawTest, RezeroWithMissingPos) {
  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 800; ++i) {
    // After 3 seconds, simulate the encoder going missing.
    // This should trigger a re-zero.  To make sure it works, change the goal as
    // well.
    if (i < 300 || i > 400) {
      claw_motor_plant_.SendPositionMessage();
    } else {
      if (i > 310) {
        // Should be re-zeroing now.
        EXPECT_TRUE(claw_motor_.is_uninitialized());
        EXPECT_TRUE(bottom_claw_motor_.is_uninitialized());
      }
      claw_queue_group.goal.MakeWithBuilder()
          .bottom_angle(0.2)
          .seperation_angle(0.2)
          .Send();
    }
    if (i == 410) {
      EXPECT_TRUE(claw_motor_.is_zeroing());
      // TODO(austin): Expose if the top and bototm is zeroing through
      // functions.
      // EXPECT_TRUE(bottom_claw_motor_.is_zeroing());
    }

    claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that disabling while zeroing sends the state machine into the
// uninitialized state.
TEST_F(ClawTest, DisableGoesUninitialized) {
  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 800; ++i) {
    claw_motor_plant_.SendPositionMessage();
    // After 0.5 seconds, disable the robot.
    if (i > 50 && i < 200) {
      SendDSPacket(false);
      if (i > 100) {
        // Give the loop a couple cycled to get the message and then verify that
        // it is in the correct state.
        EXPECT_TRUE(top_claw_motor_.is_uninitialized());
        EXPECT_TRUE(bottom_claw_motor_.is_uninitialized());
        // When disabled, we should be applying 0 power.
        EXPECT_TRUE(claw_queue_group.output.FetchLatest());
        EXPECT_EQ(0, claw_queue_group.output->top_claw_voltage);
        EXPECT_EQ(0, claw_queue_group.output->bottom_claw_voltage);
      }
    } else {
      SendDSPacket(true);
    }
    if (i == 202) {
      // Verify that we are zeroing after the bot gets enabled again.
      EXPECT_TRUE(wrist_motor_.is_zeroing());
      // TODO(austin): Expose if the top and bottom is zeroing through
      // functions.
    }

    claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
  }
  VerifyNearGoal();
}

// Tests that the wrist can't get too far away from the zeroing position if the
// zeroing position is saturating the goal.
TEST_F(ClawTest, NoWindupNegative) {
  int capped_count[2] = {0, 0};
  double saved_zeroing_position[2] = {0, 0};
  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 500; ++i) {
    claw_motor_plant_.SendPositionMessage();
    if (i == 50) {
      EXPECT_TRUE(top_claw_motor_.is_zeroing());
      EXPECT_TRUE(bottom_claw_motor_.is_zeroing());
      // Move the zeroing position far away and verify that it gets moved back.
      saved_zeroing_position[TOP_CLAW] =
          top_claw_motor_.zeroed_joint_.zeroing_position_;
      top_claw_motor_.zeroed_joint_.zeroing_position_ = -100.0;
      saved_zeroing_position[BOTTOM_CLAW] =
          bottom_claw_motor_.zeroed_joint_.zeroing_position_;
      bottom_claw_motor_.zeroed_joint_.zeroing_position_ = -100.0;
    } else if (i == 51) {
      EXPECT_TRUE(top_claw_motor_.is_zeroing());
      EXPECT_NEAR(saved_zeroing_position[TOP_CLAW],
                  top_claw_motor_.zeroed_joint_.zeroing_position_, 0.4);
      EXPECT_TRUE(bottom_claw_motor_.is_zeroing());
      EXPECT_NEAR(saved_zeroing_position[BOTTOM_CLAW],
                  bottom_claw_motor_.zeroed_joint_.zeroing_position_, 0.4);
    }
    if (!top_claw_motor_.is_ready()) {
      if (top_claw_motor_.capped_goal()) {
        ++capped_count[TOP_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(top_claw_motor_.zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(top_claw_motor_.zeroed_joint_.U_uncapped()));
      }
    }
    if (!bottom_claw_motor_.is_ready()) {
      if (bottom_claw_motor_.capped_goal()) {
        ++capped_count[BOTTOM_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(bottom_claw_motor_.zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(bottom_claw_motor_.zeroed_joint_.U_uncapped()));
      }
    }

    top_claw_motor_.Iterate();
    bottom_claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
  EXPECT_GT(3, capped_count[TOP_CLAW]);
  EXPECT_GT(3, capped_count[BOTTOM_CLAW]);
}

// Tests that the wrist can't get too far away from the zeroing position if the
// zeroing position is saturating the goal.
TEST_F(ClawTest, NoWindupPositive) {
  int capped_count[2] = {0, 0};
  double saved_zeroing_position[2] = {0, 0};
  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 500; ++i) {
    claw_motor_plant_.SendPositionMessage();
    if (i == 50) {
      EXPECT_TRUE(top_claw_motor_.is_zeroing());
      EXPECT_TRUE(top_claw_motor_.is_zeroing());
      // Move the zeroing position far away and verify that it gets moved back.
      saved_zeroing_position[TOP_CLAW] =
          top_claw_motor_.zeroed_joint_.zeroing_position_;
      top_claw_motor_.zeroed_joint_.zeroing_position_ = 100.0;
      saved_zeroing_position[BOTTOM_CLAW] =
          bottom_claw_motor_.zeroed_joint_.zeroing_position_;
      top_claw_motor_.zeroed_joint_.zeroing_position_ = 100.0;
    } else {
      if (i == 51) {
        EXPECT_TRUE(top_claw_motor_.is_zeroing());
        EXPECT_NEAR(saved_zeroing_position[TOP_CLAW],
                    top_claw_motor_.zeroed_joint_.zeroing_position_, 0.4);
        EXPECT_TRUE(bottom_claw_motor_.is_zeroing());
        EXPECT_NEAR(saved_zeroing_position[BOTTOM_CLAW],
                    bottom_claw_motor_.zeroed_joint_.zeroing_position_, 0.4);
      }
    }
    if (!top_claw_motor_.is_ready()) {
      if (top_claw_motor_.capped_goal()) {
        ++capped_count[TOP_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(top_claw_motor_.zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(top_claw_motor_.zeroed_joint_.U_uncapped()));
      }
    }
    if (!bottom_claw_motor_.is_ready()) {
      if (bottom_claw_motor_.capped_goal()) {
        ++capped_count[BOTTOM_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(bottom_claw_motor_.zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(bottom_claw_motor_.zeroed_joint_.U_uncapped()));
      }
    }

    top_claw_motor_.Iterate();
    bottom_claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
  EXPECT_GT(3, capped_count[TOP_CLAW]);
  EXPECT_GT(3, capped_count[BOTTOM_CLAW]);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
