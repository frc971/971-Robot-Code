#include <unistd.h>

#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "aos/common/network/team_number.h"
#include "aos/common/controls/control_loop_test.h"
#include "y2014/control_loops/shooter/shooter.q.h"
#include "y2014/control_loops/shooter/shooter.h"
#include "y2014/control_loops/shooter/unaugmented_shooter_motor_plant.h"
#include "y2014/constants.h"

using ::aos::time::Time;

namespace chrono = ::std::chrono;

namespace y2014 {
namespace control_loops {
namespace testing {

using ::y2014::control_loops::shooter::kMaxExtension;
using ::y2014::control_loops::shooter::MakeRawShooterPlant;

static const int kTestTeam = 1;

class TeamNumberEnvironment : public ::testing::Environment {
 public:
  // Override this to define how to set up the environment.
  virtual void SetUp() { aos::network::OverrideTeamNumber(kTestTeam); }
};

::testing::Environment *const team_number_env =
    ::testing::AddGlobalTestEnvironment(new TeamNumberEnvironment);


// Class which simulates the shooter and sends out queue messages containing the
// position.
class ShooterSimulation {
 public:
  // Constructs a motor simulation.
  ShooterSimulation(double initial_position)
      : shooter_plant_(new StateFeedbackPlant<2, 1, 1>(MakeRawShooterPlant())),
        latch_piston_state_(false),
        latch_delay_count_(0),
        plunger_latched_(false),
        brake_piston_state_(true),
        brake_delay_count_(0),
        shooter_queue_(
            ".y2014.control_loops.shooter_queue", 0xcbf22ba9,
            ".y2014.control_loops.shooter_queue.goal",
            ".y2014.control_loops.shooter_queue.position",
            ".y2014.control_loops.shooter_queue.output",
            ".y2014.control_loops.shooter_queue.status") {
    Reinitialize(initial_position);
  }

  // The difference between the position with 0 at the back, and the position
  // with 0 measured where the spring has 0 force.
  constexpr static double kPositionOffset = kMaxExtension;

  void Reinitialize(double initial_position) {
    LOG(INFO, "Reinitializing to {pos: %f}\n", initial_position);
    StateFeedbackPlant<2, 1, 1> *plant = shooter_plant_.get();
    initial_position_ = initial_position;
    plant->mutable_X(0, 0) = initial_position_ - kPositionOffset;
    plant->mutable_X(1, 0) = 0.0;
    plant->mutable_Y() = plant->C() * plant->X();
    last_voltage_ = 0.0;
    last_plant_position_ = 0.0;
    SetPhysicalSensors(&last_position_message_);
  }

  // Returns the absolute angle of the shooter.
  double GetAbsolutePosition() const {
    return shooter_plant_->Y(0, 0) + kPositionOffset;
  }

  // Returns the adjusted angle of the shooter.
  double GetPosition() const {
    return GetAbsolutePosition() - initial_position_;
  }

  // Makes sure pos is inside range (inclusive)
  bool CheckRange(double pos, struct constants::Values::AnglePair pair) {
    return (pos >= pair.lower_angle && pos <= pair.upper_angle);
  }

  // Sets the values of the physical sensors that can be directly observed
  // (encoder, hall effect).
  void SetPhysicalSensors(
      ::y2014::control_loops::ShooterQueue::Position *position) {
    const constants::Values &values = constants::GetValues();

   	position->position = GetPosition();

    LOG(DEBUG, "Physical shooter at {%f}\n", GetAbsolutePosition());

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    if (plunger_latched_) {
      position->plunger = true;
      // Only disengage the spring if we are greater than 0, which is where the
      // latch will take the load off the pusher.
      if (GetAbsolutePosition() > 0.0) {
        shooter_plant_->set_plant_index(1);
      } else {
        shooter_plant_->set_plant_index(0);
      }
    } else {
      shooter_plant_->set_plant_index(0);
      position->plunger =
          CheckRange(GetAbsolutePosition(), values.shooter.plunger_back);
    }
    position->pusher_distal.current =
        CheckRange(GetAbsolutePosition(), values.shooter.pusher_distal);
    position->pusher_proximal.current =
        CheckRange(GetAbsolutePosition(), values.shooter.pusher_proximal);
  }

  void UpdateEffectEdge(
      ::frc971::PosedgeOnlyCountedHallEffectStruct *sensor,
      const ::frc971::PosedgeOnlyCountedHallEffectStruct &last_sensor,
      const constants::Values::AnglePair &limits,
      const ::y2014::control_loops::ShooterQueue::Position &last_position) {
    sensor->posedge_count = last_sensor.posedge_count;
    sensor->negedge_count = last_sensor.negedge_count;

    sensor->posedge_value = last_sensor.posedge_value;

    if (sensor->current && !last_sensor.current) {
      ++sensor->posedge_count;
      if (last_position.position + initial_position_ < limits.lower_angle) {
        LOG(DEBUG, "Posedge value on lower edge of sensor, count is now %d\n",
            sensor->posedge_count);
        sensor->posedge_value = limits.lower_angle - initial_position_;
      } else {
        LOG(DEBUG, "Posedge value on upper edge of sensor, count is now %d\n",
            sensor->posedge_count);
        sensor->posedge_value = limits.upper_angle - initial_position_;
      }
    }
    if (!sensor->current && last_sensor.current) {
      ++sensor->negedge_count;
    }
  }

  void SendPositionMessage() {
    // the first bool is false
    SendPositionMessage(false, false, false, false);
  }

  // Sends out the position queue messages.
  // if the first bool is false then this is
  // just the default state, otherwise will force
  // it into a state using the passed values
  void SendPositionMessage(bool use_passed, bool plunger_in,
                           bool latch_in, bool brake_in) {
    const constants::Values &values = constants::GetValues();
    ::aos::ScopedMessagePtr<::y2014::control_loops::ShooterQueue::Position>
        position = shooter_queue_.position.MakeMessage();

    if (use_passed) {
      plunger_latched_ = latch_in && plunger_in;
      latch_piston_state_ = plunger_latched_;
      brake_piston_state_ = brake_in;
    }

    SetPhysicalSensors(position.get());

    position->latch = latch_piston_state_;

    // Handle pusher distal hall effect
    UpdateEffectEdge(&position->pusher_distal,
                     last_position_message_.pusher_distal,
                     values.shooter.pusher_distal, last_position_message_);

    // Handle pusher proximal hall effect
    UpdateEffectEdge(&position->pusher_proximal,
                     last_position_message_.pusher_proximal,
                     values.shooter.pusher_proximal, last_position_message_);

    last_position_message_ = *position;
    position.Send();
  }

  // Simulates the claw moving for one timestep.
  void Simulate() {
    last_plant_position_ = GetAbsolutePosition();
    EXPECT_TRUE(shooter_queue_.output.FetchLatest());
    if (shooter_queue_.output->latch_piston && !latch_piston_state_ &&
        latch_delay_count_ <= 0) {
      ASSERT_EQ(0, latch_delay_count_) << "The test doesn't support that.";
      latch_delay_count_ = 6;
    } else if (!shooter_queue_.output->latch_piston &&
               latch_piston_state_ && latch_delay_count_ >= 0) {
      ASSERT_EQ(0, latch_delay_count_) << "The test doesn't support that.";
      latch_delay_count_ = -6;
    }

    if (shooter_queue_.output->brake_piston && !brake_piston_state_ &&
        brake_delay_count_ <= 0) {
      ASSERT_EQ(0, brake_delay_count_) << "The test doesn't support that.";
      brake_delay_count_ = 5;
    } else if (!shooter_queue_.output->brake_piston &&
               brake_piston_state_ && brake_delay_count_ >= 0) {
      ASSERT_EQ(0, brake_delay_count_) << "The test doesn't support that.";
      brake_delay_count_ = -5;
    }

    // Handle brake internal state
    if (!brake_piston_state_ && brake_delay_count_ > 0) {
      if (brake_delay_count_ == 1) {
        brake_piston_state_ = true;
      }
      brake_delay_count_--;
    } else if (brake_piston_state_ && brake_delay_count_ < 0) {
      if (brake_delay_count_ == -1) {
        brake_piston_state_ = false;
      }
      brake_delay_count_++;
    }

    if (brake_piston_state_) {
      shooter_plant_->mutable_U() << 0.0;
      shooter_plant_->mutable_X(1, 0) = 0.0;
      shooter_plant_->mutable_Y() = shooter_plant_->C() * shooter_plant_->X() +
                                   shooter_plant_->D() * shooter_plant_->U();
    } else {
      shooter_plant_->mutable_U() << last_voltage_;
      //shooter_plant_->U << shooter_queue_.output->voltage;
      shooter_plant_->Update();
    }
    LOG(DEBUG, "Plant index is %d\n", shooter_plant_->plant_index());

    // Handle latch hall effect
    if (!latch_piston_state_ && latch_delay_count_ > 0) {
      LOG(DEBUG, "latching simulation: %dp\n", latch_delay_count_);
      if (latch_delay_count_ == 1) {
        latch_piston_state_ = true;
        EXPECT_GE(constants::GetValues().shooter.latch_max_safe_position,
                  GetAbsolutePosition());
        plunger_latched_ = true;
      }
      latch_delay_count_--;
    } else if (latch_piston_state_ && latch_delay_count_ < 0) {
      LOG(DEBUG, "latching simulation: %dn\n", latch_delay_count_);
      if (latch_delay_count_ == -1) {
        latch_piston_state_ = false;
        if (GetAbsolutePosition() > 0.002) {
          EXPECT_TRUE(brake_piston_state_) << "Must have the brake set when "
                                              "releasing the latch for "
                                              "powerful shots.";
        }
        plunger_latched_ = false;
        // TODO(austin): The brake should be set for a number of cycles after
        // this as well.
        shooter_plant_->mutable_X(0, 0) += 0.005;
      }
      latch_delay_count_++;
    }

    EXPECT_GE(constants::GetValues().shooter.upper_hard_limit,
              GetAbsolutePosition());
    EXPECT_LE(constants::GetValues().shooter.lower_hard_limit,
              GetAbsolutePosition());

    last_voltage_ = shooter_queue_.output->voltage;
    ::aos::time::IncrementMockTime(chrono::milliseconds(10));
  }

  // pointer to plant
  const ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> shooter_plant_;

  // true latch closed
  bool latch_piston_state_;
  // greater than zero, delaying close. less than zero delaying open
  int latch_delay_count_;

  // Goes to true after latch_delay_count_ hits 0 while the plunger is back.
  bool plunger_latched_;

  // true brake locked
  bool brake_piston_state_;
  // greater than zero, delaying close. less than zero delaying open
  int brake_delay_count_;

 private:
  ::y2014::control_loops::ShooterQueue shooter_queue_;
  double initial_position_;
  double last_voltage_;

  ::y2014::control_loops::ShooterQueue::Position last_position_message_;
  double last_plant_position_;
};

class ShooterTest : public ::aos::testing::ControlLoopTest {

 protected:
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::y2014::control_loops::ShooterQueue shooter_queue_;

  // Create a loop and simulation plant.
  ShooterMotor shooter_motor_;
  ShooterSimulation shooter_motor_plant_;

  void Reinitialize(double position) {
    shooter_motor_plant_.Reinitialize(position);
  }

  ShooterTest()
      : shooter_queue_(
            ".y2014.control_loops.shooter_queue", 0xcbf22ba9,
            ".y2014.control_loops.shooter_queue.goal",
            ".y2014.control_loops.shooter_queue.position",
            ".y2014.control_loops.shooter_queue.output",
            ".y2014.control_loops.shooter_queue.status"),
        shooter_motor_(&shooter_queue_),
        shooter_motor_plant_(0.2) {
  }

  void VerifyNearGoal() {
    shooter_queue_.goal.FetchLatest();
    shooter_queue_.position.FetchLatest();
    double pos = shooter_motor_plant_.GetAbsolutePosition();
    EXPECT_NEAR(shooter_queue_.goal->shot_power, pos, 1e-4);
  }
};

TEST_F(ShooterTest, PowerConversion) {
  const constants::Values &values = constants::GetValues();
  // test a couple of values return the right thing
  EXPECT_NEAR(0.254001, shooter_motor_.PowerToPosition(140.0), 0.00001);
  EXPECT_NEAR(0.00058, shooter_motor_.PowerToPosition(0.53), 0.00001);
  EXPECT_NEAR(0.095251238129837101, shooter_motor_.PowerToPosition(73.67),
              0.00001);

  // value too large should get max
  EXPECT_NEAR(values.shooter.upper_limit,
              shooter_motor_.PowerToPosition(505050.99), 0.00001);
  // negative values should zero
  EXPECT_NEAR(0, shooter_motor_.PowerToPosition(-123.4), 0.00001);
}

// Test that PowerToPosition and PositionToPower are inverses of each other.
// Note that PowerToPosition will cap position whereas PositionToPower will not
// cap power.
TEST_F(ShooterTest, InversePowerConversion) {
  // Test a few values.
  double power = 140.0;
  double position = shooter_motor_.PowerToPosition(power);
  EXPECT_NEAR(power, shooter_motor_.PositionToPower(position), 1e-5);
  power = .53;
  position = shooter_motor_.PowerToPosition(power);
  EXPECT_NEAR(power, shooter_motor_.PositionToPower(position), 1e-5);
  power = 71.971;
  position = shooter_motor_.PowerToPosition(power);
  EXPECT_NEAR(power, shooter_motor_.PositionToPower(position), 1e-5);
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, GoesToValue) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(2)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  // EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_queue_.goal->shot_power),
      pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, Fire) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.2)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  shooter_queue_.goal.MakeWithBuilder()
      .shot_power(35.0)
      .shot_requested(true)
      .Send();

  bool hit_fire = false;
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(5.2)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
    if (shooter_motor_.state() == ShooterMotor::STATE_FIRE) {
      if (!hit_fire) {
        shooter_queue_.goal.MakeWithBuilder()
            .shot_power(17.0)
            .shot_requested(false)
            .Send();
      }
      hit_fire = true;
    }
  }

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_queue_.goal->shot_power),
      pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  EXPECT_TRUE(hit_fire);
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, FireLong) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.5)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  shooter_queue_.goal.MakeWithBuilder().shot_requested(true).Send();

  bool hit_fire = false;
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(5.5)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
    if (shooter_motor_.state() == ShooterMotor::STATE_FIRE) {
      if (!hit_fire) {
        shooter_queue_.goal.MakeWithBuilder()
            .shot_requested(false)
            .Send();
      }
      hit_fire = true;
    }
  }

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(shooter_motor_.PowerToPosition(shooter_queue_.goal->shot_power), pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  EXPECT_TRUE(hit_fire);
}

// Verifies that it doesn't try to go out too far if you give it a ridicilous
// power.
TEST_F(ShooterTest, LoadTooFar) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(500.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.6)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
    EXPECT_LT(
        shooter_motor_plant_.GetAbsolutePosition(),
        constants::GetValuesForTeam(kTestTeam).shooter.upper_limit);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, MoveGoal) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.5)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  shooter_queue_.goal.MakeWithBuilder().shot_power(14.0).Send();

  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.0)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_queue_.goal->shot_power),
      pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}


TEST_F(ShooterTest, Unload) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.5)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  shooter_queue_.goal.MakeWithBuilder().unload_requested(true).Send();

  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(8.0) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.015);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
}

// Tests that it rezeros while unloading.
TEST_F(ShooterTest, RezeroWhileUnloading) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.5)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());

  shooter_motor_.shooter_.offset_ += 0.01;
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(2.0)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  shooter_queue_.goal.MakeWithBuilder().unload_requested(true).Send();

  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(10.0) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.015);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, UnloadWindupNegative) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.5)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  shooter_queue_.goal.MakeWithBuilder().unload_requested(true).Send();

  int kicked_delay = 20;
  int capped_goal_count = 0;
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(9.5) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    if (shooter_motor_.state() == ShooterMotor::STATE_UNLOAD_MOVE) {
      LOG(DEBUG, "State is UnloadMove\n");
      --kicked_delay;
      if (kicked_delay == 0) {
        shooter_motor_.shooter_.mutable_R(0, 0) -= 100;
      }
    }
    if (shooter_motor_.capped_goal() && kicked_delay < 0) {
      ++capped_goal_count;
    }
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
  EXPECT_LE(1, capped_goal_count);
  EXPECT_GE(3, capped_goal_count);
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, UnloadWindupPositive) {
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(1.5)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  shooter_queue_.goal.MakeWithBuilder().unload_requested(true).Send();

  int kicked_delay = 20;
  int capped_goal_count = 0;
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(9.5) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    if (shooter_motor_.state() == ShooterMotor::STATE_UNLOAD_MOVE) {
      LOG(DEBUG, "State is UnloadMove\n");
      --kicked_delay;
      if (kicked_delay == 0) {
        shooter_motor_.shooter_.mutable_R(0, 0) += 0.1;
      }
    }
    if (shooter_motor_.capped_goal() && kicked_delay < 0) {
      ++capped_goal_count;
    }
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
  EXPECT_LE(1, capped_goal_count);
  EXPECT_GE(3, capped_goal_count);
}

double HallEffectMiddle(constants::Values::AnglePair pair) {
  return (pair.lower_angle + pair.upper_angle) / 2.0;
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, StartsOnDistal) {
  Reinitialize(HallEffectMiddle(constants::GetValues().shooter.pusher_distal));
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(2.0)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  // EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_queue_.goal->shot_power),
      pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}


// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, StartsOnProximal) {
  Reinitialize(
      HallEffectMiddle(constants::GetValues().shooter.pusher_proximal));
  shooter_queue_.goal.MakeWithBuilder().shot_power(70.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(3.0)) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  // EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_queue_.goal->shot_power),
      pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

class ShooterZeroingTest : public ShooterTest,
                    public ::testing::WithParamInterface<
                        ::std::tr1::tuple<bool, bool, bool, double>> {};

TEST_P(ShooterZeroingTest, AllDisparateStartingZero) {
  bool latch = ::std::tr1::get<0>(GetParam());
  bool brake = ::std::tr1::get<1>(GetParam());
  bool plunger_back = ::std::tr1::get<2>(GetParam());
  double start_pos = ::std::tr1::get<3>(GetParam());
  // flag to initialize test
	//printf("@@@@ l= %d b= %d p= %d s= %.3f\n",
	//		latch, brake, plunger_back, start_pos);
  bool initialized = false;
  Reinitialize(start_pos);
  shooter_queue_.goal.MakeWithBuilder().shot_power(120.0).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InSeconds(2.0)) {
    shooter_motor_plant_.SendPositionMessage(!initialized, plunger_back, latch, brake);
    initialized = true;
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SimulateTimestep(true);
  }
  // EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_queue_.goal->shot_power),
      pos, 0.05);
  ASSERT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

INSTANTIATE_TEST_CASE_P(
    ShooterZeroingTest, ShooterZeroingTest,
    ::testing::Combine(
        ::testing::Bool(), ::testing::Bool(), ::testing::Bool(),
        ::testing::Values(
            0.05,
            constants::GetValuesForTeam(kTestTeam).shooter.upper_limit - 0.05,
            HallEffectMiddle(constants::GetValuesForTeam(kTestTeam)
                                 .shooter.pusher_proximal),
            HallEffectMiddle(constants::GetValuesForTeam(kTestTeam)
                                 .shooter.pusher_distal),
            constants::GetValuesForTeam(kTestTeam)
                    .shooter.latch_max_safe_position -
                0.001)));

// TODO(austin): Slip the encoder somewhere.

// TODO(austin): Test all the timeouts...

}  // namespace testing
}  // namespace control_loops
}  // namespace y2014
