#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/network/team_number.h"
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

class TeamNumberEnvironment : public ::testing::Environment {
 public:
  // Override this to define how to set up the environment.
  virtual void SetUp() { aos::network::OverrideTeamNumber(971); }
};

::testing::Environment* const team_number_env =
    ::testing::AddGlobalTestEnvironment(new TeamNumberEnvironment);


// Class which simulates the shooter and sends out queue messages containing the
// position.
class ShooterSimulation {
 public:
  // Constructs a motor simulation.
  ShooterSimulation(double initial_position)
      : shooter_plant_(new StateFeedbackPlant<2, 1, 1>(MakeRawShooterPlant())),
        shooter_queue_group_(
            ".frc971.control_loops.shooter_queue_group", 0xcbf22ba9,
            ".frc971.control_loops.shooter_queue_group.goal",
            ".frc971.control_loops.shooter_queue_group.position",
            ".frc971.control_loops.shooter_queue_group.output",
            ".frc971.control_loops.shooter_queue_group.status") {
    Reinitialize(initial_position);
  }

  void Reinitialize(double initial_position) {
    LOG(INFO, "Reinitializing to {pos: %f}\n", initial_position);
    StateFeedbackPlant<2, 1, 1> *plant = shooter_plant_.get();
    initial_position_ = initial_position;
    plant->X(0, 0) = initial_position_;
    plant->X(1, 0) = 0.0;
    plant->Y = plant->C() * plant->X;
    last_voltage_ = 0.0;
    last_plant_position_ = 0.0;
    SetPhysicalSensors(&last_position_message_);
  }

  // Returns the absolute angle of the wrist.
  double GetAbsolutePosition() const { return shooter_plant_->Y(0, 0); }

  // Returns the adjusted angle of the wrist.
  double GetPosition() const {
    return GetAbsolutePosition() - initial_position_;
  }

  // Makes sure pos is inside range (inclusive)
  bool CheckRange(double pos, struct constants::Values::AnglePair pair) {
    return (pos >= pair.lower_angle && pos <= pair.upper_angle);
  }

  // Sets the values of the physical sensors that can be directly observed
  // (encoder, hall effect).
  void SetPhysicalSensors(control_loops::ShooterGroup::Position *position) {
    const frc971::constants::Values &values = constants::GetValues();
    position->position = GetAbsolutePosition();

    LOG(DEBUG, "Physical shooter at {%f}\n", position->position);

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    position->plunger.current =
        CheckRange(position->position, values.shooter.plunger_back);
    position->pusher_distal.current =
        CheckRange(position->position, values.shooter.pusher_distal);
    position->pusher_proximal.current =
        CheckRange(position->position, values.shooter.pusher_proximal);
  }

  void UpdateEffectEdge(HallEffectStruct *sensor,
                        const HallEffectStruct &last_sensor,
                        const constants::Values::AnglePair &limits,
                        control_loops::ShooterGroup::Position *position,
                        const control_loops::ShooterGroup::Position &last_position) {
    if (sensor->current && !last_sensor.current) {
      ++sensor->posedge_count;
      if (last_position.position < limits.lower_angle) {
        position->pusher_posedge_value = limits.lower_angle - initial_position_;
      } else {
        position->pusher_posedge_value = limits.upper_angle - initial_position_;
      }
    }
    if (!sensor->current && last_sensor.current) {
      ++sensor->negedge_count;
      if (position->position < limits.lower_angle) {
        position->pusher_negedge_value = limits.lower_angle - initial_position_;
      } else {
        position->pusher_negedge_value = limits.upper_angle - initial_position_;
      }
    }
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const frc971::constants::Values &values = constants::GetValues();
    ::aos::ScopedMessagePtr<control_loops::ShooterGroup::Position> position =
        shooter_queue_group_.position.MakeMessage();

    SetPhysicalSensors(position.get());

    // Handle latch hall effect
    if (!latch_piston_state_ && latch_delay_count_ > 0) {
      LOG(DEBUG, "latching simulation: %dp\n", latch_delay_count_);
      if (latch_delay_count_ == 1) {
        latch_piston_state_ = true;
        position->latch.current = true;
      }
      latch_delay_count_--;
    } else if (latch_piston_state_ && latch_delay_count_ < 0) {
      LOG(DEBUG, "latching simulation: %dn\n", latch_delay_count_);
      if (latch_delay_count_ == -1) {
        latch_piston_state_ = false;
        position->latch.current = false;
      }
      latch_delay_count_++;
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

    // Handle plunger hall effect
    UpdateEffectEdge(&position->plunger,
                     last_position_message_.plunger,
                     values.shooter.plunger_back,
                     position.get(),
                     last_position_message_);
    LOG(INFO, "seteffect: plunger back: %d\n",
        position->plunger.current);

    // Handle pusher distal hall effect
    UpdateEffectEdge(&position->pusher_distal,
                     last_position_message_.pusher_distal,
                     values.shooter.pusher_distal,
                     position.get(),
                     last_position_message_);
    LOG(INFO, "seteffect: pusher distal: %d\n",
        position->plunger.current);

    // Handle pusher proximal hall effect
    UpdateEffectEdge(&position->pusher_proximal,
                     last_position_message_.pusher_proximal,
                     values.shooter.pusher_proximal,
                     position.get(),
                     last_position_message_);
    LOG(INFO, "seteffect: pusher proximal: %d\n",
        position->plunger.current);

    last_position_message_ = *position;
    position.Send();
  }

  // Simulates the claw moving for one timestep.
  void Simulate() {
    last_plant_position_ = shooter_plant_->Y(0, 0);
    EXPECT_TRUE(shooter_queue_group_.output.FetchLatest());
    shooter_plant_->U << last_voltage_;
    shooter_plant_->Update();
    if (shooter_queue_group_.output->latch_piston && !latch_piston_state_ &&
        latch_delay_count_ == 0) {
      latch_delay_count_ = 6;
    } else if (!shooter_queue_group_.output->latch_piston &&
               latch_piston_state_ && latch_delay_count_ == 0) {
      latch_delay_count_ = -6;
    }

    if (shooter_queue_group_.output->brake_piston && !brake_piston_state_ &&
        brake_delay_count_ == 0) {
      brake_delay_count_ = 5;
    } else if (!shooter_queue_group_.output->brake_piston &&
               brake_piston_state_ && brake_delay_count_ == 0) {
      brake_delay_count_ = -5;
    }

    EXPECT_GE(constants::GetValues().shooter.upper_limit,
              shooter_plant_->Y(0, 0));
    // we okay within a millimeter
    EXPECT_LE(constants::GetValues().shooter.lower_limit - 1.0,
              shooter_plant_->Y(0, 0));
    last_voltage_ = shooter_queue_group_.output->voltage;
    ::aos::time::Time::IncrementMockTime(::aos::time::Time::InMS(10.0));
  }

  // pointer to plant
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> shooter_plant_;

  // true latch closed
  int latch_piston_state_;
  // greater than zero, delaying close. less than zero delaying open
  int latch_delay_count_;

  // true brake locked
  int brake_piston_state_;
  // greater than zero, delaying close. less than zero delaying open
  int brake_delay_count_;

 private:

  ShooterGroup shooter_queue_group_;
  double initial_position_;
  double last_voltage_;

  control_loops::ShooterGroup::Position last_position_message_;
  double last_plant_position_;
};

class ShooterTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ShooterGroup shooter_queue_group_;

  // Create a loop and simulation plant.
  ShooterMotor shooter_motor_;
  ShooterSimulation shooter_motor_plant_;

  ShooterTest()
      : shooter_queue_group_(
            ".frc971.control_loops.shooter_queue_group", 0xcbf22ba9,
            ".frc971.control_loops.shooter_queue_group.goal",
            ".frc971.control_loops.shooter_queue_group.position",
            ".frc971.control_loops.shooter_queue_group.output",
            ".frc971.control_loops.shooter_queue_group.status"),
        shooter_motor_(&shooter_queue_group_),
        shooter_motor_plant_(0.5) {
    // Flush the robot state queue so we can use clean shared memory for this
    // test.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
    ::bbb::sensor_generation.Clear();
    ::bbb::sensor_generation.MakeWithBuilder()
        .reader_pid(254)
        .cape_resets(5)
        .Send();
    ::aos::time::Time::EnableMockTime(::aos::time::Time::InSeconds(0.0));
  }

  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled).autonomous(false)
        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

  void VerifyNearGoal() {
    shooter_queue_group_.goal.FetchLatest();
    shooter_queue_group_.position.FetchLatest();
    double pos = shooter_motor_plant_.GetAbsolutePosition();
    EXPECT_NEAR(shooter_queue_group_.goal->shot_power, pos, 1e-4);
  }

  virtual ~ShooterTest() {
    ::aos::robot_state.Clear();
    ::bbb::sensor_generation.Clear();
    ::aos::time::Time::DisableMockTime();
  }
};

TEST_F(ShooterTest, PowerConversion) {
  // test a couple of values return the right thing
  EXPECT_EQ(0.021, shooter_motor_.PowerToPosition(0.021));
  EXPECT_EQ(0.475, shooter_motor_.PowerToPosition(0.475));

  const frc971::constants::Values &values = constants::GetValues();
  // value too large should get max
  EXPECT_EQ(values.shooter.upper_limit,
            shooter_motor_.PowerToPosition(505050.99));
  // negative values should zero
  EXPECT_EQ(0.0, shooter_motor_.PowerToPosition(-123.4));
}

// Tests that the wrist zeros correctly and goes to a position.
TEST_F(ShooterTest, GoesToValue) {
  shooter_queue_group_.goal.MakeWithBuilder().shot_power(0.21).Send();
  for (int i = 0; i < 100; ++i) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  //EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(shooter_queue_group_.goal->shot_power, pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.GetState());
}

// Tests that the wrist zeros correctly and goes to a position.
TEST_F(ShooterTest, Fire) {
  shooter_queue_group_.goal.MakeWithBuilder().shot_power(0.021).Send();
  for (int i = 0; i < 100; ++i) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.GetState());
  shooter_queue_group_.goal.MakeWithBuilder().shot_requested(true).Send();

  bool hit_requestfire = false;
  bool hit_preparefire = false;
  bool hit_fire = false;
  for (int i = 0; i < 100; ++i) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
	printf("MOTORSTATE = %d\n", shooter_motor_.GetState());
	if (shooter_motor_.GetState() == ShooterMotor::STATE_REQUEST_FIRE){
		hit_requestfire = true;
	}
	if (shooter_motor_.GetState() == ShooterMotor::STATE_PREPARE_FIRE){
		hit_preparefire = true;
	}
	if (shooter_motor_.GetState() == ShooterMotor::STATE_FIRE){
		hit_fire = true;
	}
  }

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_NEAR(shooter_queue_group_.goal->shot_power, pos, 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.GetState());
  EXPECT_TRUE(hit_requestfire);
  EXPECT_TRUE(hit_preparefire);
  EXPECT_TRUE(hit_fire);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
