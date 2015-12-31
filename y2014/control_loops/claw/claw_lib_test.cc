#include <unistd.h>

#include <memory>

#include "aos/common/network/team_number.h"
#include "aos/common/controls/control_loop_test.h"

#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/control_loops/claw/claw.h"
#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw_motor_plant.h"

#include "gtest/gtest.h"


namespace y2014 {
namespace control_loops {
namespace testing {

using ::y2014::control_loops::claw::MakeClawPlant;
using ::frc971::HallEffectStruct;
using ::y2014::control_loops::HalfClawPosition;

typedef enum {
	TOP_CLAW = 0,
	BOTTOM_CLAW,

	CLAW_COUNT
} ClawType;

class TeamNumberEnvironment : public ::testing::Environment {
 public:
  // Override this to define how to set up the environment.
  virtual void SetUp() { aos::network::OverrideTeamNumber(1); }
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
        claw_queue(".y2014.control_loops.claw_queue", 0x9f1a99dd,
                   ".y2014.control_loops.claw_queue.goal",
                   ".y2014.control_loops.claw_queue.position",
                   ".y2014.control_loops.claw_queue.output",
                   ".y2014.control_loops.claw_queue.status") {
    Reinitialize(initial_top_position, initial_bottom_position);
  }

  void Reinitialize(double initial_top_position,
                    double initial_bottom_position) {
    LOG(INFO, "Reinitializing to {top: %f, bottom: %f}\n", initial_top_position,
        initial_bottom_position);
    claw_plant_->mutable_X(0, 0) = initial_bottom_position;
    claw_plant_->mutable_X(1, 0) = initial_top_position - initial_bottom_position;
    claw_plant_->mutable_X(2, 0) = 0.0;
    claw_plant_->mutable_X(3, 0) = 0.0;
    claw_plant_->mutable_Y() = claw_plant_->C() * claw_plant_->X();

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

  // Makes sure pos is inside range (exclusive)
  bool CheckRange(double pos, const constants::Values::Claws::AnglePair &pair) {
    // Note: If the >= and <= signs are used, then the there exists a case
    // where the wrist starts precisely on the edge and because initial
    // position and the *edge_value_ are the same, then the comparison
    // in ZeroedStateFeedbackLoop::DoGetPositionOfEdge will return that
    // the lower, rather than upper, edge of the hall effect was passed.
    return (pos > pair.lower_angle && pos < pair.upper_angle);
  }

  void SetHallEffect(double pos,
                     const constants::Values::Claws::AnglePair &pair,
                     HallEffectStruct *hall_effect) {
    hall_effect->current = CheckRange(pos, pair);
  }

  void SetClawHallEffects(double pos,
                          const constants::Values::Claws::Claw &claw,
                          HalfClawPosition *half_claw) {
    SetHallEffect(pos, claw.front, &half_claw->front);
    SetHallEffect(pos, claw.calibration, &half_claw->calibration);
    SetHallEffect(pos, claw.back, &half_claw->back);
  }

  // Sets the values of the physical sensors that can be directly observed
  // (encoder, hall effect).
  void SetPhysicalSensors(
      ::y2014::control_loops::ClawQueue::Position *position) {
    position->top.position = GetPosition(TOP_CLAW);
    position->bottom.position = GetPosition(BOTTOM_CLAW);

    double pos[2] = {GetAbsolutePosition(TOP_CLAW),
                     GetAbsolutePosition(BOTTOM_CLAW)};
    LOG(DEBUG, "Physical claws are at {top: %f, bottom: %f}\n", pos[TOP_CLAW],
        pos[BOTTOM_CLAW]);

    const constants::Values& values = constants::GetValues();

    // Signal that each hall effect sensor has been triggered if it is within
    // the correct range.
    SetClawHallEffects(pos[TOP_CLAW], values.claw.upper_claw, &position->top);
    SetClawHallEffects(pos[BOTTOM_CLAW], values.claw.lower_claw,
                       &position->bottom);
  }

  void UpdateHallEffect(double angle,
                        double last_angle,
                        double initial_position,
                        HallEffectStruct *position,
                        const HallEffectStruct &last_position,
                        const constants::Values::Claws::AnglePair &pair,
                        const char *claw_name, const char *hall_effect_name) {
    if (position->current && !last_position.current) {
      ++position->posedge_count;

      if (last_angle < pair.lower_angle) {
        LOG(DEBUG, "%s: Positive lower edge on %s hall effect\n", claw_name,
            hall_effect_name);
        position->posedge_value = pair.lower_angle - initial_position;
      } else {
        LOG(DEBUG, "%s: Positive upper edge on %s hall effect\n", claw_name,
            hall_effect_name);
        position->posedge_value = pair.upper_angle - initial_position;
      }
    }
    if (!position->current && last_position.current) {
      ++position->negedge_count;

      if (angle < pair.lower_angle) {
        LOG(DEBUG, "%s: Negative lower edge on %s hall effect\n", claw_name,
            hall_effect_name);
        position->negedge_value = pair.lower_angle - initial_position;
      } else {
        LOG(DEBUG, "%s: Negative upper edge on %s hall effect\n", claw_name,
            hall_effect_name);
        position->negedge_value = pair.upper_angle - initial_position;
      }
    }
  }

  void UpdateClawHallEffects(
      HalfClawPosition *position,
      const HalfClawPosition &last_position,
      const constants::Values::Claws::Claw &claw, double initial_position,
      const char *claw_name) {
    UpdateHallEffect(position->position + initial_position,
                     last_position.position + initial_position,
                     initial_position, &position->front, last_position.front,
                     claw.front, claw_name, "front");
    UpdateHallEffect(position->position + initial_position,
                     last_position.position + initial_position,
                     initial_position, &position->calibration,
                     last_position.calibration, claw.calibration, claw_name,
                     "calibration");
    UpdateHallEffect(position->position + initial_position,
                     last_position.position + initial_position,
                     initial_position, &position->back, last_position.back,
                     claw.back, claw_name, "back");
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<::y2014::control_loops::ClawQueue::Position>
        position = claw_queue.position.MakeMessage();

    // Initialize all the counters to their previous values.
    *position = last_position_;

    SetPhysicalSensors(position.get());

    const constants::Values& values = constants::GetValues();

    UpdateClawHallEffects(&position->top, last_position_.top,
                          values.claw.upper_claw, initial_position_[TOP_CLAW],
                          "Top");
    UpdateClawHallEffects(&position->bottom, last_position_.bottom,
                          values.claw.lower_claw,
                          initial_position_[BOTTOM_CLAW], "Bottom");

    // Only set calibration if it changed last cycle.  Calibration starts out
    // with a value of 0.
    last_position_ = *position;
    position.Send();
  }

  // Simulates the claw moving for one timestep.
  void Simulate() {
    const constants::Values& v = constants::GetValues();
    EXPECT_TRUE(claw_queue.output.FetchLatest());

    claw_plant_->mutable_U() << claw_queue.output->bottom_claw_voltage,
        claw_queue.output->top_claw_voltage;
    claw_plant_->Update();

    // Check that the claw is within the limits.
    EXPECT_GE(v.claw.upper_claw.upper_limit, claw_plant_->Y(0, 0));
    EXPECT_LE(v.claw.upper_claw.lower_hard_limit, claw_plant_->Y(0, 0));

    EXPECT_GE(v.claw.lower_claw.upper_hard_limit, claw_plant_->Y(1, 0));
    EXPECT_LE(v.claw.lower_claw.lower_hard_limit, claw_plant_->Y(1, 0));

    EXPECT_LE(claw_plant_->Y(1, 0) - claw_plant_->Y(0, 0),
              v.claw.claw_max_separation);
    EXPECT_GE(claw_plant_->Y(1, 0) - claw_plant_->Y(0, 0),
              v.claw.claw_min_separation);
  }
  // The whole claw.
  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> claw_plant_;

 private:
  // Resets the plant so that it starts at initial_position.
  void ReinitializePartial(ClawType type, double initial_position) {
    initial_position_[type] = initial_position;
  }

  ::y2014::control_loops::ClawQueue claw_queue;
  double initial_position_[CLAW_COUNT];

  ::y2014::control_loops::ClawQueue::Position last_position_;
};


class ClawTest : public ::aos::testing::ControlLoopTest {
 protected:
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::y2014::control_loops::ClawQueue claw_queue;

  // Create a loop and simulation plant.
  ClawMotor claw_motor_;
  ClawMotorSimulation claw_motor_plant_;

  // Minimum amount of acceptable separation between the top and bottom of the
  // claw.
  double min_separation_;

  ClawTest()
      : claw_queue(".y2014.control_loops.claw_queue", 0x9f1a99dd,
                   ".y2014.control_loops.claw_queue.goal",
                   ".y2014.control_loops.claw_queue.position",
                   ".y2014.control_loops.claw_queue.output",
                   ".y2014.control_loops.claw_queue.status"),
        claw_motor_(&claw_queue),
        claw_motor_plant_(0.4, 0.2),
        min_separation_(constants::GetValues().claw.claw_min_separation) {}

  void VerifyNearGoal() {
    claw_queue.goal.FetchLatest();
    claw_queue.position.FetchLatest();
    double bottom = claw_motor_plant_.GetAbsolutePosition(BOTTOM_CLAW);
    double separation =
        claw_motor_plant_.GetAbsolutePosition(TOP_CLAW) - bottom;
    EXPECT_NEAR(claw_queue.goal->bottom_angle, bottom, 1e-4);
    EXPECT_NEAR(claw_queue.goal->separation_angle, separation, 1e-4);
    EXPECT_LE(min_separation_, separation);
  }
};

TEST_F(ClawTest, HandlesNAN) {
  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(::std::nan(""))
      .separation_angle(::std::nan(""))
      .Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(5)) {
    claw_motor_plant_.SendPositionMessage();
    claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
}

// Tests that the wrist zeros correctly and goes to a position.
TEST_F(ClawTest, ZerosCorrectly) {
  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .separation_angle(0.2)
      .Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(5)) {
    claw_motor_plant_.SendPositionMessage();
    claw_motor_.Iterate();
    claw_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  VerifyNearGoal();
}

// Tests that the wrist zeros correctly and goes to a position.
TEST_F(ClawTest, LimitClawGoal) {
  constants::Values values;
  values.claw.claw_min_separation = -2.0;
  values.claw.claw_max_separation = 1.0;
  values.claw.soft_min_separation = -2.0;
  values.claw.soft_max_separation = 1.0;
  values.claw.upper_claw.lower_limit = -5.0;
  values.claw.upper_claw.upper_limit = 7.5;
  values.claw.lower_claw.lower_limit = -5.5;
  values.claw.lower_claw.upper_limit = 8.0;

  double bottom_goal = 0.0;
  double top_goal = 0.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(0.0, bottom_goal, 1e-4);
  EXPECT_NEAR(0.0, top_goal, 1e-4);

  bottom_goal = 0.0;
  top_goal = -4.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(-1.0, bottom_goal, 1e-4);
  EXPECT_NEAR(-3.0, top_goal, 1e-4);

  bottom_goal = 0.0;
  top_goal = 4.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(1.5, bottom_goal, 1e-4);
  EXPECT_NEAR(2.5, top_goal, 1e-4);

  bottom_goal = -10.0;
  top_goal = -9.5;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(-5.5, bottom_goal, 1e-4);
  EXPECT_NEAR(-5.0, top_goal, 1e-4);

  bottom_goal = -20.0;
  top_goal = -4.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(-5.5, bottom_goal, 1e-4);
  EXPECT_NEAR(-4.5, top_goal, 1e-4);

  bottom_goal = -20.0;
  top_goal = -4.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(-5.5, bottom_goal, 1e-4);
  EXPECT_NEAR(-4.5, top_goal, 1e-4);

  bottom_goal = -5.0;
  top_goal = -10.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(-3.0, bottom_goal, 1e-4);
  EXPECT_NEAR(-5.0, top_goal, 1e-4);

  bottom_goal = 10.0;
  top_goal = 9.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(8.0, bottom_goal, 1e-4);
  EXPECT_NEAR(7.0, top_goal, 1e-4);

  bottom_goal = 8.0;
  top_goal = 9.0;

  LimitClawGoal(&bottom_goal, &top_goal, values);
  EXPECT_NEAR(6.5, bottom_goal, 1e-4);
  EXPECT_NEAR(7.5, top_goal, 1e-4);
}


class ZeroingClawTest
    : public ClawTest,
      public ::testing::WithParamInterface< ::std::pair<double, double>> {};

// Tests that the wrist zeros correctly starting on the hall effect sensor.
TEST_P(ZeroingClawTest, ParameterizedZero) {
  claw_motor_plant_.Reinitialize(GetParam().first, GetParam().second);

  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .separation_angle(0.2)
      .Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(7)) {
    claw_motor_plant_.SendPositionMessage();
    claw_motor_.Iterate();
    claw_motor_plant_.Simulate();

    SimulateTimestep(true);
  }
  VerifyNearGoal();
}

INSTANTIATE_TEST_CASE_P(ZeroingClawTest, ZeroingClawTest,
                        ::testing::Values(::std::make_pair(0.04, 0.02),
                                          ::std::make_pair(0.2, 0.1),
                                          ::std::make_pair(0.3, 0.2),
                                          ::std::make_pair(0.4, 0.3),
                                          ::std::make_pair(0.5, 0.4),
                                          ::std::make_pair(0.6, 0.5),
                                          ::std::make_pair(0.7, 0.6),
                                          ::std::make_pair(0.8, 0.7),
                                          ::std::make_pair(0.9, 0.8),
                                          ::std::make_pair(1.0, 0.9),
                                          ::std::make_pair(1.1, 1.0),
                                          ::std::make_pair(1.15, 1.05),
                                          ::std::make_pair(1.05, 0.95),
                                          ::std::make_pair(1.2, 1.1),
                                          ::std::make_pair(1.3, 1.2),
                                          ::std::make_pair(1.4, 1.3),
                                          ::std::make_pair(1.5, 1.4),
                                          ::std::make_pair(1.6, 1.5),
                                          ::std::make_pair(1.7, 1.6),
                                          ::std::make_pair(1.8, 1.7),
                                          ::std::make_pair(2.015, 2.01)
));

/*
// Tests that loosing the encoder for a second triggers a re-zero.
TEST_F(ClawTest, RezeroWithMissingPos) {
  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .separation_angle(0.2)
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
      claw_queue.goal.MakeWithBuilder()
          .bottom_angle(0.2)
          .separation_angle(0.2)
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
    SimulateTimestep(true);
  }
  VerifyNearGoal();
}

// Tests that disabling while zeroing sends the state machine into the
// uninitialized state.
TEST_F(ClawTest, DisableGoesUninitialized) {
  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .separation_angle(0.2)
      .Send();
  for (int i = 0; i < 800; ++i) {
    claw_motor_plant_.SendPositionMessage();
    // After 0.5 seconds, disable the robot.
    if (i > 50 && i < 200) {
      SimulateTimestep(false);
      if (i > 100) {
        // Give the loop a couple cycled to get the message and then verify that
        // it is in the correct state.
        EXPECT_TRUE(claw_motor_.is_uninitialized());
        // When disabled, we should be applying 0 power.
        EXPECT_TRUE(claw_queue.output.FetchLatest());
        EXPECT_EQ(0, claw_queue.output->top_claw_voltage);
        EXPECT_EQ(0, claw_queue.output->bottom_claw_voltage);
      }
    } else {
      SimulateTimestep(true);
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
*/

class WindupClawTest : public ClawTest {
 protected:
  void TestWindup(ClawMotor::CalibrationMode mode, ::aos::time::Time start_time, double offset) {
    int capped_count = 0;
    const constants::Values& values = constants::GetValues();
    bool kicked = false;
    bool measured = false;
    while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(7)) {
      claw_motor_plant_.SendPositionMessage();
      if (::aos::time::Time::Now() >= start_time &&
          mode == claw_motor_.mode() && !kicked) {
        EXPECT_EQ(mode, claw_motor_.mode());
        // Move the zeroing position far away and verify that it gets moved
        // back.
        claw_motor_.top_claw_goal_ += offset;
        claw_motor_.bottom_claw_goal_ += offset;
        kicked = true;
      } else {
        if (kicked && !measured) {
          measured = true;
          EXPECT_EQ(mode, claw_motor_.mode());

          Eigen::Matrix<double, 4, 1> R;
          R << claw_motor_.bottom_claw_goal_,
              claw_motor_.top_claw_goal_ - claw_motor_.bottom_claw_goal_, 0.0,
              0.0;
          Eigen::Matrix<double, 2, 1> uncapped_voltage =
              claw_motor_.claw_.K() * (R - claw_motor_.claw_.X_hat());
          // Use a factor of 1.8 because so long as it isn't actually running
          // away, the CapU function will deal with getting the actual output
          // down.
          EXPECT_LT(::std::abs(uncapped_voltage(0, 0)),
                    values.claw.max_zeroing_voltage * 1.8);
          EXPECT_LT(::std::abs(uncapped_voltage(1, 0)),
                    values.claw.max_zeroing_voltage * 1.8);
        }
      }
      if (claw_motor_.mode() == mode) {
        if (claw_motor_.capped_goal()) {
          ++capped_count;
          // The cycle after we kick the zero position is the only cycle during
          // which we should expect to see a high uncapped power during zeroing.
          ASSERT_LT(values.claw.max_zeroing_voltage,
                    ::std::abs(claw_motor_.uncapped_average_voltage()));
        } else {
          ASSERT_GT(values.claw.max_zeroing_voltage,
                    ::std::abs(claw_motor_.uncapped_average_voltage()));
        }
      }

      claw_motor_.Iterate();
      claw_motor_plant_.Simulate();
      SimulateTimestep(true);
    }
    EXPECT_TRUE(kicked);
    EXPECT_TRUE(measured);
    EXPECT_LE(1, capped_count);
    EXPECT_GE(3, capped_count);
  }
};

// Tests that the wrist can't get too far away from the zeroing position if the
// zeroing position is saturating the goal.
TEST_F(WindupClawTest, NoWindupPositive) {
  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .separation_angle(0.2)
      .Send();

  TestWindup(ClawMotor::UNKNOWN_LOCATION, ::aos::time::Time::InSeconds(1.0),
             971.0);

  VerifyNearGoal();
}

// Tests that the wrist can't get too far away from the zeroing position if the
// zeroing position is saturating the goal.
TEST_F(WindupClawTest, NoWindupNegative) {
  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .separation_angle(0.2)
      .Send();

  TestWindup(ClawMotor::UNKNOWN_LOCATION, ::aos::time::Time::InSeconds(1.0),
             -971.0);

  VerifyNearGoal();
}

// Tests that the wrist can't get too far away from the zeroing position if the
// zeroing position is saturating the goal.
TEST_F(WindupClawTest, NoWindupNegativeFineTune) {
  claw_queue.goal.MakeWithBuilder()
      .bottom_angle(0.1)
      .separation_angle(0.2)
      .Send();

  TestWindup(ClawMotor::FINE_TUNE_BOTTOM, ::aos::time::Time::InSeconds(2.0),
             -971.0);

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace control_loops
}  // namespace y2014
