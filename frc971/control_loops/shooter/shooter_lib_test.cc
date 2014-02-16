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
      : shooter_plant_(new StateFeedbackPlant<2, 1, 1>(MakeRawTopClawPlant()))
        shooter_queue_group(".frc971.control_loops.claw_queue_group", 0x9f1a99dd,
                         ".frc971.control_loops.claw_queue_group.goal",
                         ".frc971.control_loops.claw_queue_group.position",
                         ".frc971.control_loops.claw_queue_group.output",
                         ".frc971.control_loops.claw_queue_group.status") {
    Reinitialize(initial_position);
  }


  void Reinitialize(double initial_position) {
    LOG(INFO, "Reinitializing to {top: %f}\n", initial_position);
    StateFeedbackPlant<2, 1, 1>* plant = shooter_plant_.get();
    initial_position_ = initial_position;
    plant->X(0, 0) = initial_position_;
    plant->X(1, 0) = 0.0;
    plant->Y = plant->C() * plant->X;
    last_voltage_ = 0.0;
    last_position_.Zero();
    SetPhysicalSensors(&last_position_);
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
  bool CheckRange(double pos, struct constants::Values::AnglePair pair) {
    return (pos >= pair.lower_angle && pos <= pair.upper_angle);
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
        CheckRange(position->position, values.plunger_heffect);
    position->pusher_distal_hall_effect =
        CheckRange(position->position, values.pusher_distal_heffect);
    position->pusher_proximal_hall_effect =
        CheckRange(position->position, values.pusher_proximal_heffect);
    position->latch_hall_effect =
        CheckRange(position->position, values.latch_heffect);
  }


  void UpdateEffectEdge(bool effect, bool last_effect,
		  double upper_angle, double lower_angle, double position,
  		   double &posedge_value, double &negedge_value,
		  int &posedge_count, int &negedge_count) {
	  if (effect && !last_effect) {
	  	  ++posedge_count;
		  if (last_position_.position < lower_angle) {
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
			negedge_value - upper_angle - initial_position_;
		}
	}
  }


  // Sends out the position queue messages.
  void SendPositionMessage() {
    const frc971::constants::Values& values = constants::GetValues();
    ::aos::ScopedMessagePtr<control_loops::ShooterGroup::Position> position =
        shooter_queue_group.position.MakeMessage();

    // Initialize all the counters to their previous values.
    *position = last_position_;

    SetPhysicalSensors(position.get());

    // Handle the front hall effect.
    if (position->plunger_back_hall_effect &&
        !last_position_.plunger_back_hall_effect) {
      ++position->plunger_back_hall_effect_posedge_count;

      if (last_position_.position < values.upper_claw.front.lower_angle) {
        position->top.posedge_value =
            values.upper_claw.front.lower_angle - initial_position_;
      } else {
        position->top.posedge_value =
            values.upper_claw.front.upper_angle - initial_position_;
      }
    }
    if (!position->plunger_back_hall_effect &&
        last_position_.plunger_back_hall_effect) {
      ++position->plunger_back_hall_effect_negedge_count;

      if (position->top.position < values.upper_claw.front.lower_angle) {
        position->top.negedge_value =
            values.upper_claw.front.lower_angle - initial_position_;
      } else {
        position->top.negedge_value =
            values.upper_claw.front.upper_angle - initial_position_;
      }
    }

	// Handle plunger hall effect
	UpdateEffectEdge(
			position->plunger_back_hall_effect,
			last_position_.plunger_back_hall_effect,
			values.plunger_back.upper_angle,
			values.plunger_back.lower_angle,
			position->position,
			position->posedge_value,
			position->negedge_value,
			position->plunger_back_hall_effect_posedge_count,
			position->plunger_back_hall_effect_negedge_count);

	// Handle pusher distal hall effect
	UpdateEffectEdge(
			position->pusher_distal_hall_effect,
			last_position_.pusher_distal_hall_effect,
			values.pusher_distal.upper_angle,
			values.pusher_distal.lower_angle,
			position->position,
			position->posedge_value,
			position->negedge_value,
			position->pusher_distal_hall_effect_posedge_count,
			position->pusher_distal_hall_effect_negedge_count);

	// Handle pusher proximal hall effect
	UpdateEffectEdge(
			position->pusher_proximal_hall_effect,
			last_position_.pusher_proximal_hall_effect,
			values.pusher_proximal.upper_angle,
			values.pusher_proximal.lower_angle,
			position->position,
			position->posedge_value,
			position->negedge_value,
			position->pusher_proximal_hall_effect_posedge_count,
			position->pusher_proximal_hall_effect_negedge_count);

	// Handle latch hall effect
	UpdateEffectEdge(
			position->latch_hall_effect,
			last_position_.latch_hall_effect,
			values.latch.upper_angle,
			values.latch.lower_angle,
			position->position,
			position->posedge_value,
			position->negedge_value,
			position->latch_hall_effect_posedge_count,
			position->latch_hall_effect_negedge_count);

    // Only set calibration if it changed last cycle.  Calibration starts out
    // with a value of 0.
    last_position_ = *position;
  }


  // Simulates the claw moving for one timestep.
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


  // Top of the claw, the one with rollers
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> shooter_plant_;


 private:

  ShooterGroup shooter_queue_group;
  double initial_position_;
  double last_voltage_;

  control_loops::ShooterGroup::Position last_position_;
};


class ShooterTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ShooterGroup shooter_queue_group;

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
    shooter_queue_group.goal.FetchLatest();
    shooter_queue_group.position.FetchLatest();
    double pos = shooter_motor_plant_.GetAbsolutePosition();
    EXPECT_NEAR(shooter_queue_group.goal->shot_power, pos, 1e-4);
  }

  virtual ~ShooterTest() {
    ::aos::robot_state.Clear();
  }
};


// Tests that the wrist zeros correctly and goes to a position.
TEST_F(ShooterTest, ZerosCorrectly) {
  shooter_queue_group.goal.MakeWithBuilder()
      .shot_power(5050.1)
      .Send();
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
