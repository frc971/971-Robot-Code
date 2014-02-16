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
class ShooterSimulation {
 public:
  // Constructs a motor simulation.
  ShooterSimulation(double initial_position)
      : shooter_plant_(new StateFeedbackPlant<3, 1, 1>(MakeShooterPlant())),
        shooter_queue_group_(
            ".frc971.control_loops.shooter_queue_group", 0xcbf22ba9,
            ".frc971.control_loops.shooter_queue_group.goal",
            ".frc971.control_loops.shooter_queue_group.position",
            ".frc971.control_loops.shooter_queue_group.output",
            ".frc971.control_loops.shooter_queue_group.status") {
    Reinitialize(initial_position);
  }
  void Reinitialize(double initial_position) {
    LOG(INFO, "Reinitializing to {top: %f}\n", initial_position);
    StateFeedbackPlant<3, 1, 1>* plant = shooter_plant_.get();
    initial_position_ = initial_position;
    plant->X(0, 0) = initial_position_;
    plant->X(1, 0) = 0.0;
    plant->Y = plant->C() * plant->X;
    last_voltage_ = 0.0;
    last_plant_position_ = 0.0;
    SetPhysicalSensors(&last_position_message_);
  }


  // Returns the absolute angle of the wrist.
  double GetAbsolutePosition() const {
      return shooter_plant_->Y(0,0);
  }


  // Returns the adjusted angle of the wrist.
  double GetPosition() const {
    return GetAbsolutePosition() - initial_position_;
  }


  // Makes sure pos is inside range (inclusive)
  bool CheckRange(double pos, struct constants::Values::Pair pair) {
    return (pos >= pair.lower_limit && pos <= pair.upper_limit);
  }


  // Sets the values of the physical sensors that can be directly observed
  // (encoder, hall effect).
  void SetPhysicalSensors(control_loops::ShooterGroup::Position *position) {
    const frc971::constants::Values& values = constants::GetValues();
    position->position = GetPosition();

    LOG(DEBUG, "Physical shooter at {%f}\n", position->position);

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    position->plunger_back_hall_effect =
        CheckRange(position->position, values.shooter.plunger_back);
    position->pusher_distal_hall_effect =
        CheckRange(position->position, values.shooter.pusher_distal);
    position->pusher_proximal_hall_effect =
        CheckRange(position->position, values.shooter.pusher_proximal);
  }

  void UpdateEffectEdge(bool effect, bool last_effect, double upper_angle,
                        double lower_angle, double position,
                        double &posedge_value, double &negedge_value,
                        int64_t &posedge_count, int64_t &negedge_count) {
   if (effect && !last_effect) {
	  	  ++posedge_count;
		  if (last_position_message_.position < lower_angle) {
		  	  posedge_value = lower_angle - initial_position_;
		  } else {
		  	  posedge_value = upper_angle - initial_position_;
		  }
	  }

	if (!effect && last_effect) {
		++negedge_count;
		if (position < lower_angle) {
			negedge_value = lower_angle - initial_position_;
		} else {
			negedge_value = upper_angle - initial_position_;
		}
	}
  }


  // Sends out the position queue messages.
  void SendPositionMessage() {
    const frc971::constants::Values& values = constants::GetValues();
    ::aos::ScopedMessagePtr<control_loops::ShooterGroup::Position> position =
        shooter_queue_group_.position.MakeMessage();

    SetPhysicalSensors(position.get());

	// Handle plunger hall effect
    UpdateEffectEdge(position->plunger_back_hall_effect,
                     last_position_message_.plunger_back_hall_effect,
                     values.shooter.plunger_back.upper_limit,
                     values.shooter.plunger_back.lower_limit,
                     position->position, position->posedge_value,
                     position->negedge_value,
                     position->plunger_back_hall_effect_posedge_count,
                     position->plunger_back_hall_effect_negedge_count);

 // Handle pusher distal hall effect
    UpdateEffectEdge(position->pusher_distal_hall_effect,
                     last_position_message_.pusher_distal_hall_effect,
                     values.shooter.pusher_distal.upper_limit,
                     values.shooter.pusher_distal.lower_limit,
                     position->position, position->posedge_value,
                     position->negedge_value,
                     position->pusher_distal_hall_effect_posedge_count,
                     position->pusher_distal_hall_effect_negedge_count);

 // Handle pusher proximal hall effect
    UpdateEffectEdge(position->pusher_proximal_hall_effect,
                     last_position_message_.pusher_proximal_hall_effect,
                     values.shooter.pusher_proximal.upper_limit,
                     values.shooter.pusher_proximal.lower_limit,
                     position->position, position->posedge_value,
                     position->negedge_value,
                     position->pusher_proximal_hall_effect_posedge_count,
                     position->pusher_proximal_hall_effect_negedge_count);

    last_position_message_ = *position;
    position.Send();
  }


  // Simulates the claw moving for one timestep.
  void Simulate() {
    last_plant_position_ = shooter_plant_->Y(0, 0);
    EXPECT_TRUE(shooter_queue_group_.output.FetchLatest());
    shooter_plant_->U << last_voltage_;
    shooter_plant_->Update();

    EXPECT_GE(constants::GetValues().shooter.upper_limit,
              shooter_plant_->Y(0, 0));
    EXPECT_LE(constants::GetValues().shooter.lower_limit,
              shooter_plant_->Y(0, 0));
    last_voltage_ = shooter_queue_group_.output->voltage;
  }


  // pointer to plant
  ::std::unique_ptr<StateFeedbackPlant<3, 1, 1>> shooter_plant_;


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
  }


  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled)
                                        .autonomous(false)
                                        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

  void VerifyNearGoal() {
    shooter_queue_group_.goal.FetchLatest();
    shooter_queue_group_.position.FetchLatest();
    double pos = shooter_motor_plant_.GetAbsolutePosition();
    EXPECT_NEAR(shooter_queue_group_.goal->shot_power, pos, 1e-4);
  }

  virtual ~ShooterTest() { ::aos::robot_state.Clear(); }
};


TEST_F(ShooterTest, PowerConversion) {
  // test a couple of values return the right thing
  EXPECT_EQ(2.1, shooter_motor_.PowerToPosition(2.1));
  EXPECT_EQ(50.99, shooter_motor_.PowerToPosition(50.99));

  const frc971::constants::Values &values = constants::GetValues();
  // value too large should get max
  EXPECT_EQ(values.shooter.upper_limit,
            shooter_motor_.PowerToPosition(505050.99));
  // negative values should zero
  EXPECT_EQ(0.0, shooter_motor_.PowerToPosition(-123.4));
}

// Tests that the wrist zeros correctly and goes to a position.
TEST_F(ShooterTest, ZerosCorrectly) {
  shooter_queue_group_.goal.MakeWithBuilder().shot_power(2.1).Send();
  for (int i = 0; i < 400; ++i) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
