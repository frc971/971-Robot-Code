#include <unistd.h>

#include <memory>

#include "aos/common/network/team_number.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw.h"
#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw_motor_plant.h"

#include "gtest/gtest.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

typedef enum {
	TOP_CLAW = 0,
	BOTTOM_CLAW,

	CLAW_COUNT
} ClawType;

class TeamNumberEnvironment : public ::testing::Environment {
 public:
  // Override this to define how to set up the environment.
  virtual void SetUp() { aos::network::OverrideTeamNumber(971); }
};

::testing::Environment* const team_number_env =
    ::testing::AddGlobalTestEnvironment(new TeamNumberEnvironment);

// Class which simulates the wrist and sends out queue messages containing the
// position.
class ClawMotorSimulation {
 public:
  // Constructs a motor simulation.  initial_position is the inital angle of the
  // wrist, which will be treated as 0 by the encoder.
  ClawMotorSimulation(double initial_top_position,
                      double initial_bottom_position)
      : claw_plant_(new StateFeedbackPlant<4, 2, 2>(MakeClawPlant())),
        claw_queue_group(".frc971.control_loops.claw_queue_group", 0x9f1a99dd,
                         ".frc971.control_loops.claw_queue_group.goal",
                         ".frc971.control_loops.claw_queue_group.position",
                         ".frc971.control_loops.claw_queue_group.output",
                         ".frc971.control_loops.claw_queue_group.status") {
    Reinitialize(initial_top_position, initial_bottom_position);
  }

  void Reinitialize(double initial_top_position,
                    double initial_bottom_position) {
    LOG(INFO, "Reinitializing to {top: %f, bottom: %f}\n", initial_top_position,
        initial_bottom_position);
    claw_plant_->X(0, 0) = initial_bottom_position;
    claw_plant_->X(1, 0) = initial_top_position - initial_bottom_position;
    claw_plant_->X(2, 0) = 0.0;
    claw_plant_->X(3, 0) = 0.0;
    claw_plant_->Y = claw_plant_->C() * claw_plant_->X;

    ReinitializePartial(TOP_CLAW, initial_top_position);
    ReinitializePartial(BOTTOM_CLAW, initial_bottom_position);
    last_position_.Zero();
    SetPhysicalSensors(&last_position_);
  }

  // Returns the absolute angle of the wrist.
  double GetAbsolutePosition(ClawType type) const {
    if (type == TOP_CLAW) {
      return claw_plant_->Y(1, 0);
    } else {
      return claw_plant_->Y(0, 0);
    }
  }

  // Returns the adjusted angle of the wrist.
  double GetPosition(ClawType type) const {
    return GetAbsolutePosition(type) - initial_position_[type];
  }

  // Makes sure pos is inside range (inclusive)
  bool CheckRange(double pos, struct constants::Values::AnglePair pair) {
    return (pos >= pair.lower_angle && pos <= pair.upper_angle);
  }

  // Sets the values of the physical sensors that can be directly observed
  // (encoder, hall effect).
  void SetPhysicalSensors(control_loops::ClawGroup::Position *position) {
    position->top.position = GetPosition(TOP_CLAW);
    position->bottom.position = GetPosition(BOTTOM_CLAW);

    double pos[2] = {GetAbsolutePosition(TOP_CLAW),
                     GetAbsolutePosition(BOTTOM_CLAW)};
    LOG(DEBUG, "Physical claws are at {top: %f, bottom: %f}\n", pos[TOP_CLAW], pos[BOTTOM_CLAW]);

    const frc971::constants::Values& values = constants::GetValues();

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    position->top.front_hall_effect =
        CheckRange(pos[TOP_CLAW], values.upper_claw.front);
    position->top.calibration_hall_effect =
        CheckRange(pos[TOP_CLAW], values.upper_claw.calibration);
    position->top.back_hall_effect =
        CheckRange(pos[TOP_CLAW], values.upper_claw.back);

    position->bottom.front_hall_effect =
        CheckRange(pos[BOTTOM_CLAW], values.lower_claw.front);
    position->bottom.calibration_hall_effect =
        CheckRange(pos[BOTTOM_CLAW], values.lower_claw.calibration);
    position->bottom.back_hall_effect =
        CheckRange(pos[BOTTOM_CLAW], values.lower_claw.back);
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::ClawGroup::Position> position =
        claw_queue_group.position.MakeMessage();

    // Initialize all the counters to their previous values.
    *position = last_position_;

    SetPhysicalSensors(position.get());

    const frc971::constants::Values& values = constants::GetValues();
    double last_top_position =
        last_position_.top.position + initial_position_[TOP_CLAW];
    double last_bottom_position =
        last_position_.bottom.position + initial_position_[BOTTOM_CLAW];
    double top_position =
        position->top.position + initial_position_[TOP_CLAW];
    double bottom_position =
        position->bottom.position + initial_position_[BOTTOM_CLAW];

    // Handle the front hall effect.
    if (position->top.front_hall_effect &&
        !last_position_.top.front_hall_effect) {
      ++position->top.front_hall_effect_posedge_count;

      if (last_top_position < values.upper_claw.front.lower_angle) {
        LOG(DEBUG, "Top: Positive lower edge front hall effect\n");
        position->top.posedge_value =
            values.upper_claw.front.lower_angle - initial_position_[TOP_CLAW];
      } else {
        LOG(DEBUG, "Top: Positive upper edge front hall effect\n");
        position->top.posedge_value =
            values.upper_claw.front.upper_angle - initial_position_[TOP_CLAW];
      }
    }
    if (!position->top.front_hall_effect &&
        last_position_.top.front_hall_effect) {
      ++position->top.front_hall_effect_negedge_count;

      if (top_position < values.upper_claw.front.lower_angle) {
        LOG(DEBUG, "Top: Negative lower edge front hall effect\n");
        position->top.negedge_value =
            values.upper_claw.front.lower_angle - initial_position_[TOP_CLAW];
      } else {
        LOG(DEBUG, "Top: Negative upper edge front hall effect\n");
        position->top.negedge_value =
            values.upper_claw.front.upper_angle - initial_position_[TOP_CLAW];
      }
    }

    // Handle the calibration hall effect.
    if (position->top.calibration_hall_effect &&
        !last_position_.top.calibration_hall_effect) {
      ++position->top.calibration_hall_effect_posedge_count;

      if (last_top_position < values.upper_claw.calibration.lower_angle) {
        LOG(DEBUG, "Top: Positive lower edge calibration hall effect\n");
        position->top.posedge_value =
            values.upper_claw.calibration.lower_angle -
            initial_position_[TOP_CLAW];
      } else {
        LOG(DEBUG, "Top: Positive upper edge calibration hall effect\n");
        position->top.posedge_value =
            values.upper_claw.calibration.upper_angle - initial_position_[TOP_CLAW];
      }
    }
    if (!position->top.calibration_hall_effect &&
        last_position_.top.calibration_hall_effect) {
      ++position->top.calibration_hall_effect_negedge_count;

      if (top_position < values.upper_claw.calibration.lower_angle) {
        LOG(DEBUG, "Top: Negative lower edge calibration hall effect\n");
        position->top.negedge_value =
            values.upper_claw.calibration.lower_angle - initial_position_[TOP_CLAW];
      } else {
        LOG(DEBUG, "Top: Negative upper edge calibration hall effect\n");
        position->top.negedge_value =
            values.upper_claw.calibration.upper_angle - initial_position_[TOP_CLAW];
      }
    }

    // Handle the back hall effect.
    if (position->top.back_hall_effect &&
        !last_position_.top.back_hall_effect) {
      ++position->top.back_hall_effect_posedge_count;

      if (last_top_position < values.upper_claw.back.lower_angle) {
        LOG(DEBUG, "Top: Positive lower edge back hall effect\n");
        position->top.posedge_value =
            values.upper_claw.back.lower_angle - initial_position_[TOP_CLAW];
      } else {
        LOG(DEBUG, "Top: Positive upper edge back hall effect\n");
        position->top.posedge_value =
            values.upper_claw.back.upper_angle - initial_position_[TOP_CLAW];
      }
    }
    if (!position->top.back_hall_effect &&
        last_position_.top.back_hall_effect) {
      ++position->top.back_hall_effect_negedge_count;

      if (top_position < values.upper_claw.back.lower_angle) {
        LOG(DEBUG, "Top: Negative upper edge back hall effect\n");
        position->top.negedge_value =
            values.upper_claw.back.lower_angle - initial_position_[TOP_CLAW];
      } else {
        LOG(DEBUG, "Top: Negative lower edge back hall effect\n");
        position->top.negedge_value =
            values.upper_claw.back.upper_angle - initial_position_[TOP_CLAW];
      }
    }

    // Now deal with the bottom part of the claw.
    // Handle the front hall effect.
    if (position->bottom.front_hall_effect &&
        !last_position_.bottom.front_hall_effect) {
      ++position->bottom.front_hall_effect_posedge_count;

      if (last_bottom_position < values.lower_claw.front.lower_angle) {
        LOG(DEBUG, "Bottom: Positive lower edge front hall effect\n");
        position->bottom.posedge_value =
            values.lower_claw.front.lower_angle - initial_position_[BOTTOM_CLAW];
      } else {
        LOG(DEBUG, "Bottom: Positive upper edge front hall effect\n");
        position->bottom.posedge_value =
            values.lower_claw.front.upper_angle - initial_position_[BOTTOM_CLAW];
      }
    }
    if (!position->bottom.front_hall_effect &&
        last_position_.bottom.front_hall_effect) {
      ++position->bottom.front_hall_effect_negedge_count;

      if (bottom_position < values.lower_claw.front.lower_angle) {
        LOG(DEBUG, "Bottom: Negative lower edge front hall effect\n");
        position->bottom.negedge_value =
            values.lower_claw.front.lower_angle - initial_position_[BOTTOM_CLAW];
      } else {
        LOG(DEBUG, "Bottom: Negative upper edge front hall effect\n");
        position->bottom.negedge_value =
            values.lower_claw.front.upper_angle - initial_position_[BOTTOM_CLAW];
      }
    }

    // Handle the calibration hall effect.
    if (position->bottom.calibration_hall_effect &&
        !last_position_.bottom.calibration_hall_effect) {
      ++position->bottom.calibration_hall_effect_posedge_count;

      if (last_bottom_position < values.lower_claw.calibration.lower_angle) {
        LOG(DEBUG, "Bottom: Positive lower edge calibration hall effect\n");
        position->bottom.posedge_value =
            values.lower_claw.calibration.lower_angle -
            initial_position_[BOTTOM_CLAW];
      } else {
        LOG(DEBUG, "Bottom: Positive upper edge calibration hall effect\n");
        position->bottom.posedge_value =
            values.lower_claw.calibration.upper_angle -
            initial_position_[BOTTOM_CLAW];
      }
    }
    if (!position->bottom.calibration_hall_effect &&
        last_position_.bottom.calibration_hall_effect) {
      ++position->bottom.calibration_hall_effect_negedge_count;

      if (bottom_position < values.lower_claw.calibration.lower_angle) {
        LOG(DEBUG, "Bottom: Negative lower edge calibration hall effect\n");
        position->bottom.negedge_value =
            values.lower_claw.calibration.lower_angle -
            initial_position_[BOTTOM_CLAW];
      } else {
        LOG(DEBUG, "Bottom: Negative upper edge calibration hall effect\n");
        position->bottom.negedge_value =
            values.lower_claw.calibration.upper_angle -
            initial_position_[BOTTOM_CLAW];
      }
    }

    // Handle the back hall effect.
    if (position->bottom.back_hall_effect &&
        !last_position_.bottom.back_hall_effect) {
      ++position->bottom.back_hall_effect_posedge_count;

      if (last_bottom_position < values.lower_claw.back.lower_angle) {
        LOG(DEBUG, "Bottom: Positive lower edge back hall effect\n");
        position->bottom.posedge_value =
            values.lower_claw.back.lower_angle - initial_position_[BOTTOM_CLAW];
      } else {
        LOG(DEBUG, "Bottom: Positive upper edge back hall effect\n");
        position->bottom.posedge_value =
            values.lower_claw.back.upper_angle - initial_position_[BOTTOM_CLAW];
      }
    }
    if (!position->bottom.back_hall_effect &&
        last_position_.bottom.back_hall_effect) {
      ++position->bottom.back_hall_effect_negedge_count;

      if (bottom_position < values.lower_claw.back.lower_angle) {
        LOG(DEBUG, "Bottom: Negative lower edge back hall effect\n");
        position->bottom.negedge_value =
            values.lower_claw.back.lower_angle - initial_position_[BOTTOM_CLAW];
      } else {
        LOG(DEBUG, "Bottom: Negative upper edge back hall effect\n");
        position->bottom.negedge_value =
            values.lower_claw.back.upper_angle - initial_position_[BOTTOM_CLAW];
      }
    }

    // Only set calibration if it changed last cycle.  Calibration starts out
    // with a value of 0.
    last_position_ = *position;
    position.Send();
  }

  // Simulates the claw moving for one timestep.
  void Simulate() {
    const frc971::constants::Values& v = constants::GetValues();
    EXPECT_TRUE(claw_queue_group.output.FetchLatest());

    claw_plant_->U << claw_queue_group.output->bottom_claw_voltage,
        claw_queue_group.output->top_claw_voltage -
            claw_queue_group.output->bottom_claw_voltage;
    claw_plant_->Update();

    // Check that the claw is within the limits.
    EXPECT_GE(v.upper_claw.upper_limit, claw_plant_->Y(0, 0));
    EXPECT_LE(v.upper_claw.lower_limit, claw_plant_->Y(0, 0));

    EXPECT_GE(v.lower_claw.upper_limit, claw_plant_->Y(1, 0));
    EXPECT_LE(v.lower_claw.lower_limit, claw_plant_->Y(1, 0));

    EXPECT_LE(claw_plant_->Y(1, 0) - claw_plant_->Y(0, 0),
              v.claw_max_seperation);
    EXPECT_GE(claw_plant_->Y(1, 0) - claw_plant_->Y(0, 0),
              v.claw_min_seperation);
  }
  // The whole claw.
  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> claw_plant_;

 private:
  // Resets the plant so that it starts at initial_position.
  void ReinitializePartial(ClawType type, double initial_position) {
    initial_position_[type] = initial_position;
  }

  ClawGroup claw_queue_group;
  double initial_position_[CLAW_COUNT];

  control_loops::ClawGroup::Position last_position_;
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
      : claw_queue_group(".frc971.control_loops.claw_queue_group", 0x9f1a99dd,
                         ".frc971.control_loops.claw_queue_group.goal",
                         ".frc971.control_loops.claw_queue_group.position",
                         ".frc971.control_loops.claw_queue_group.output",
                         ".frc971.control_loops.claw_queue_group.status"),
        claw_motor_(&claw_queue_group),
        claw_motor_plant_(0.4, 0.2),
        min_seperation_(constants::GetValues().claw_min_seperation) {
    // Flush the robot state queue so we can use clean shared memory for this
    // test.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
  }

  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder()
        .enabled(enabled)
        .autonomous(false)
        .team_id(971)
        .Send();
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
    claw_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that the wrist zeros correctly starting on the hall effect sensor.
TEST_F(ClawTest, ZerosStartingOn) {
  claw_motor_plant_.Reinitialize(100 * M_PI / 180.0, 90 * M_PI / 180.0);

  claw_queue_group.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .seperation_angle(0.2)
      .Send();
  for (int i = 0; i < 500; ++i) {
    claw_motor_plant_.SendPositionMessage();
    claw_motor_.Iterate();
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
  for (int i = 0; i < 500; ++i) {
    if (i % 23) {
      claw_motor_plant_.SendPositionMessage();
    }
    claw_motor_.Iterate();
    claw_motor_plant_.Simulate();

    SendDSPacket(true);
  }
  VerifyNearGoal();
}

/*
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
        EXPECT_TRUE(claw_motor_.is_uninitialized());
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
      EXPECT_TRUE(claw_motor_.is_zeroing());
      // Move the zeroing position far away and verify that it gets moved back.
      saved_zeroing_position[TOP_CLAW] =
          top_claw_motor_.zeroed_joint_.zeroing_position_;
      top_claw_motor_.zeroed_joint_.zeroing_position_ = -100.0;
      saved_zeroing_position[BOTTOM_CLAW] =
          bottom_claw_motor_.zeroed_joint_.zeroing_position_;
      bottom_claw_motor_.zeroed_joint_.zeroing_position_ = -100.0;
    } else if (i == 51) {
      EXPECT_TRUE(claw_motor_.is_zeroing());

      EXPECT_NEAR(saved_zeroing_position[TOP_CLAW],
                  top_claw_motor_.zeroed_joint_.zeroing_position_, 0.4);
      EXPECT_NEAR(saved_zeroing_position[BOTTOM_CLAW],
                  bottom_claw_motor_.zeroed_joint_.zeroing_position_, 0.4);
    }
    if (!claw_motor_.top().is_ready()) {
      if (claw_motor_.top().capped_goal()) {
        ++capped_count[TOP_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(top_claw_motor_.zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(top_claw_motor_.zeroed_joint_.U_uncapped()));
      }
    }
    if (!claw_motor_.bottom().is_ready()) {
      if (claw_motor_.bottom().capped_goal()) {
        ++capped_count[BOTTOM_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(bottom_claw_motor_.zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(bottom_claw_motor_.zeroed_joint_.U_uncapped()));
      }
    }

    claw_motor_.Iterate();
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
        EXPECT_TRUE(claw_motor_.top().is_zeroing());
        EXPECT_TRUE(claw_motor_.bottom().is_zeroing());
        EXPECT_NEAR(saved_zeroing_position[TOP_CLAW],
                    claw_motor_.top().zeroing_position_, 0.4);
        EXPECT_NEAR(saved_zeroing_position[BOTTOM_CLAW],
                    claw_motor_.bottom().zeroing_position_, 0.4);
      }
    }
    if (!top_claw_motor_.is_ready()) {
      if (top_claw_motor_.capped_goal()) {
        ++capped_count[TOP_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(claw_motor_.top().zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(claw_motor_.top().zeroed_joint_.U_uncapped()));
      }
    }
    if (!bottom_claw_motor_.is_ready()) {
      if (bottom_claw_motor_.capped_goal()) {
        ++capped_count[BOTTOM_CLAW];
        // The cycle after we kick the zero position is the only cycle during
        // which we should expect to see a high uncapped power during zeroing.
        ASSERT_LT(5, ::std::abs(claw_motor_.bottom().zeroed_joint_.U_uncapped()));
      } else {
        ASSERT_GT(5, ::std::abs(claw_motor_.bottom().zeroed_joint_.U_uncapped()));
      }
    }

    claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
  EXPECT_GT(3, capped_count[TOP_CLAW]);
  EXPECT_GT(3, capped_count[BOTTOM_CLAW]);
}
*/

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
