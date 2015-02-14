#include <math.h>
#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "aos/common/controls/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/fridge/fridge.h"
#include "frc971/constants.h"
#include "frc971/control_loops/team_number_test_environment.h"

using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {
// Class which simulates the fridge and sends out queue messages with the
// position.
class FridgeSimulation {
 public:
  static constexpr double kNoiseScalar = 3.0;
  // Constructs a simulation.
  FridgeSimulation()
      : arm_plant_(new StateFeedbackPlant<4, 2, 2>(MakeArmPlant())),
        elev_plant_(new StateFeedbackPlant<4, 2, 2>(MakeElevatorPlant())),
        left_arm_pot_encoder_(
            constants::GetValues().left_arm_zeroing_constants.index_difference),
        right_arm_pot_encoder_(
            constants::GetValues()
                .right_arm_zeroing_constants.index_difference),
        left_elev_pot_encoder_(
            constants::GetValues()
                .left_elevator_zeroing_constants.index_difference),
        right_elev_pot_encoder_(
            constants::GetValues()
                .right_elevator_zeroing_constants.index_difference),
        fridge_queue_(".frc971.control_loops.fridge_queue", 0xe4e05855,
                      ".frc971.control_loops.fridge_queue.goal",
                      ".frc971.control_loops.fridge_queue.position",
                      ".frc971.control_loops.fridge_queue.output",
                      ".frc971.control_loops.fridge_queue.status") {
    // Initialize the elevator.
    SetElevatorSensors(
        constants::GetValues().fridge.elevator.lower_limit,
        constants::GetValues().fridge.elevator.lower_limit,
        kNoiseScalar *
            constants::GetValues()
                .right_elevator_zeroing_constants.index_difference);
    // Initialize the arm.
    SetArmSensors(0.0, 0.0,
                  kNoiseScalar *
                      constants::GetValues()
                          .right_arm_zeroing_constants.index_difference);
  }

  void SetElevatorSensors(double left_start_pos, double right_start_pos,
                          double pot_noise_stddev) {
    // Update the internal state of the elevator plant.
    elev_plant_->mutable_X(0, 0) = (left_start_pos + right_start_pos) / 2.0;
    elev_plant_->mutable_X(2, 0) = (left_start_pos - right_start_pos) / 2.0;

    right_elev_pot_encoder_.Initialize(right_start_pos, pot_noise_stddev);
    left_elev_pot_encoder_.Initialize(left_start_pos, pot_noise_stddev);
  }

  void SetArmSensors(double left_start_pos, double right_start_pos,
                     double pot_noise_stddev) {
    // Update the internal state of the arm plant.
    arm_plant_->mutable_X(0, 0) = (left_start_pos + right_start_pos) / 2.0;
    arm_plant_->mutable_X(2, 0) = (left_start_pos - right_start_pos) / 2.0;

    left_arm_pot_encoder_.Initialize(left_start_pos, pot_noise_stddev);
    right_arm_pot_encoder_.Initialize(right_start_pos, pot_noise_stddev);
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::FridgeQueue::Position> position =
        fridge_queue_.position.MakeMessage();

    left_arm_pot_encoder_.GetSensorValues(&position->arm.left);
    right_arm_pot_encoder_.GetSensorValues(&position->arm.right);
    left_elev_pot_encoder_.GetSensorValues(&position->elevator.left);
    right_elev_pot_encoder_.GetSensorValues(&position->elevator.right);

    position.Send();
  }

  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(fridge_queue_.output.FetchLatest());

    // Feed voltages into physics simulation.
    arm_plant_->mutable_U() << fridge_queue_.output->left_arm,
        fridge_queue_.output->right_arm;
    elev_plant_->mutable_U() << fridge_queue_.output->left_elevator,
        fridge_queue_.output->right_elevator;

    // Use the plant to generate the next physical state given the voltages to
    // the motors.
    arm_plant_->Update();
    elev_plant_->Update();

    const double left_arm_angle = arm_plant_->Y(0, 0);
    const double right_arm_angle = arm_plant_->Y(1, 0);
    const double left_elev_height = elev_plant_->Y(0, 0);
    const double right_elev_height = elev_plant_->Y(1, 0);

    // TODO (phil) Do some sanity tests on the arm angles and the elevator
    // heights. For example, we need to make sure that both sides are within a
    // certain distance of each other and they haven't crashed into the top or
    // the bottom.

    // Use the physical state to simulate sensor readings.
    left_arm_pot_encoder_.MoveTo(left_arm_angle);
    right_arm_pot_encoder_.MoveTo(right_arm_angle);
    left_elev_pot_encoder_.MoveTo(left_elev_height);
    right_elev_pot_encoder_.MoveTo(right_elev_height);
  }

  void VerifySeparation() {
    // TODO(danielp): Make sure that we are getting the correct values from the
    // plant.
    const double left_arm_angle = arm_plant_->Y(0, 0);
    const double right_arm_angle = arm_plant_->Y(1, 0);
    const double left_elev_height = elev_plant_->Y(0, 0);
    const double right_elev_height = elev_plant_->Y(1, 0);

    EXPECT_NEAR(left_arm_angle, right_arm_angle, 5.0 * M_PI / 180.0);
    EXPECT_NEAR(left_elev_height, right_elev_height, 0.05);
  }

 private:
  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> arm_plant_;
  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> elev_plant_;

  PositionSensorSimulator left_arm_pot_encoder_;
  PositionSensorSimulator right_arm_pot_encoder_;
  PositionSensorSimulator left_elev_pot_encoder_;
  PositionSensorSimulator right_elev_pot_encoder_;

  FridgeQueue fridge_queue_;
};

class FridgeTest : public ::aos::testing::ControlLoopTest {
 protected:
  FridgeTest()
      : fridge_queue_(".frc971.control_loops.fridge_queue", 0xe4e05855,
                      ".frc971.control_loops.fridge_queue.goal",
                      ".frc971.control_loops.fridge_queue.position",
                      ".frc971.control_loops.fridge_queue.output",
                      ".frc971.control_loops.fridge_queue.status"),
        fridge_(&fridge_queue_),
        fridge_plant_() {
    set_team_id(kTeamNumber);
  }

  void VerifyNearGoal() {
    fridge_queue_.goal.FetchLatest();
    fridge_queue_.status.FetchLatest();
    EXPECT_NEAR(fridge_queue_.goal->angle, fridge_queue_.status->angle, 0.001);
    EXPECT_NEAR(fridge_queue_.goal->height, fridge_queue_.status->height,
                0.001);
  }

  // Runs one iteration of the whole simulation and checks that separation
  // remains reasonable.
  void RunIteration() {
    SendMessages(true);

    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();

    TickTime();

    fridge_plant_.VerifySeparation();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const Time &run_for) {
    const auto start_time = Time::Now();
    while (Time::Now() < start_time + run_for) {
      RunIteration();
    }
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  FridgeQueue fridge_queue_;

  // Create a control loop and simulation.
  Fridge fridge_;
  FridgeSimulation fridge_plant_;
};

// Tests that the loop does nothing when the goal is zero.
TEST_F(FridgeTest, DoesNothing) {
  // Set the goals to the starting values. This should theoretically guarantee
  // that the controller does nothing.
  const constants::Values values = constants::GetValues();
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
      .angle(0.0)
      .height(values.fridge.elevator.lower_limit)
      .Send());

  // Run a few iterations.
  RunForTime(Time::InMS(5000));

  VerifyNearGoal();
}

// Tests that the loop can reach a goal.
TEST_F(FridgeTest, ReachesGoal) {
  // Set a reasonable goal.
  const auto values = constants::GetValues();
  const double soft_limit = values.fridge.elevator.lower_limit;
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .height(soft_limit + 0.2)
      .Send());

  // Give it a lot of time to get there.
  RunForTime(Time::InMS(4000));

  VerifyNearGoal();
}

// Tests that the loop doesn't try and go beyond the physical range of the
// mechanisms.
TEST_F(FridgeTest, RespectsRange) {
  // Put the arm up to get it out of the way.
  // We're going to send the elevator to zero, which should be significantly too
  // low.
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
      .angle(M_PI)
      .height(-5.0)
      .Send());

  RunForTime(Time::InMS(4000));

  // Check that we are near our soft limit.
  fridge_queue_.status.FetchLatest();
  const auto values = constants::GetValues();
  EXPECT_NEAR(values.fridge.elevator.lower_limit, fridge_queue_.status->height,
              0.001);
  EXPECT_NEAR(values.fridge.arm.upper_limit, fridge_queue_.status->angle,
              0.001);

  // Put the arm down to get it out of the way.
  // We're going to give the elevator some ridiculously high goal.
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
      .angle(-M_PI)
      .height(50.0)
      .Send());

  RunForTime(Time::InMS(4000));

  // Check that we are near our soft limit.
  fridge_queue_.status.FetchLatest();
  EXPECT_NEAR(values.fridge.elevator.upper_limit, fridge_queue_.status->height,
              0.001);
  EXPECT_NEAR(values.fridge.arm.lower_limit, fridge_queue_.status->angle,
              0.001);
}

// Tests that the loop zeroes when run for a while.
TEST_F(FridgeTest, ZeroTest) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.0).height(0.5).Send();
  while (::aos::time::Time::Now() < ::aos::time::Time::InMS(4000)) {
    SendMessages(true);
    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();
    TickTime();
  }

  VerifyNearGoal();
}

// TODO(austin): Update unit test to abort when limits are violated.
// TODO(austin): Test that voltages are within 12 volt ranges.

// TODO(austin): Test starting with nonzero encoder values.
// TODO(austin): Test that we ignore DMA counts during the uninitialized period
// TODO(austin): Test starting with nonzero DMA counts.

// TODO(austin): Test starting at all 4 hard limits.

// TODO(austin): Check that we don't move the zeroing goals if disabled.
// TODO(austin): Test that windup works correctly for both the arm and elevator.
// TODO(austin): Check that we e-stop if the joints start too far apart.

// Nice to have below here.

// TODO(austin): Check that we e-stop if encoder index pulse is not n revolutions away from last one. (got extra counts from noise, etc).
// TODO(austin): Check that we e-stop if pot disagrees too much with encoder after we are zeroed.


}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
