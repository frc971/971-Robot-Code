#include "y2015/control_loops/fridge/fridge.h"

#include <math.h>
#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/common/commonmath.h"
#include "aos/common/controls/control_loop_test.h"
#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2015/constants.h"
#include "y2015/control_loops/fridge/arm_motor_plant.h"
#include "y2015/control_loops/fridge/elevator_motor_plant.h"
#include "y2015/control_loops/fridge/fridge.q.h"
#include "y2015/util/kinematics.h"

namespace y2015 {
namespace control_loops {
namespace fridge {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

// Class which simulates the fridge and sends out queue messages with the
// position.
class FridgeSimulation {
 public:
  static constexpr double kNoiseScalar = 0.1;
  // Constructs a simulation.
  FridgeSimulation()
      : arm_plant_(new StateFeedbackPlant<4, 2, 2>(MakeArmPlant())),
        elevator_plant_(new StateFeedbackPlant<4, 2, 2>(MakeElevatorPlant())),
        left_arm_pot_encoder_(
            constants::GetValues().fridge.left_arm_zeroing.index_difference),
        right_arm_pot_encoder_(
            constants::GetValues().fridge.right_arm_zeroing.index_difference),
        left_elevator_pot_encoder_(
            constants::GetValues().fridge.left_elev_zeroing.index_difference),
        right_elevator_pot_encoder_(
            constants::GetValues().fridge.right_elev_zeroing.index_difference),
        fridge_queue_(".y2015.control_loops.fridge.fridge_queue", 0xe4e05855,
                      ".y2015.control_loops.fridge.fridge_queue.goal",
                      ".y2015.control_loops.fridge.fridge_queue.position",
                      ".y2015.control_loops.fridge.fridge_queue.output",
                      ".y2015.control_loops.fridge.fridge_queue.status") {
    // Initialize the elevator.
    InitializeElevatorPosition(
        constants::GetValues().fridge.elevator.lower_limit);
    // Initialize the arm.
    InitializeArmPosition(0.0);
  }

  void InitializeElevatorPosition(double start_pos) {
    InitializeElevatorPosition(start_pos, start_pos);
  }

  void InitializeElevatorPosition(double left_start_pos,
                                  double right_start_pos) {
    InitializeElevatorPosition(
        left_start_pos, right_start_pos,
        kNoiseScalar *
            constants::GetValues().fridge.right_elev_zeroing.index_difference);
  }

  void InitializeElevatorPosition(double left_start_pos, double right_start_pos,
                                  double pot_noise_stddev) {
    // Update the internal state of the elevator plant.
    elevator_plant_->mutable_X(0, 0) = (left_start_pos + right_start_pos) / 2.0;
    elevator_plant_->mutable_X(1, 0) = 0.0;
    elevator_plant_->mutable_X(2, 0) = (left_start_pos - right_start_pos) / 2.0;
    elevator_plant_->mutable_X(3, 0) = 0.0;

    right_elevator_pot_encoder_.Initialize(right_start_pos, pot_noise_stddev);
    left_elevator_pot_encoder_.Initialize(left_start_pos, pot_noise_stddev);
    elevator_initial_difference_ = left_start_pos - right_start_pos;
  }

  void InitializeArmPosition(double start_pos) {
    InitializeArmPosition(start_pos, start_pos);
  }

  void InitializeArmPosition(double left_start_pos, double right_start_pos) {
    InitializeArmPosition(
        left_start_pos, right_start_pos,
        kNoiseScalar *
            constants::GetValues().fridge.right_arm_zeroing.index_difference);
  }
  void InitializeArmPosition(double left_start_pos, double right_start_pos,
                             double pot_noise_stddev) {
    // Update the internal state of the arm plant.
    arm_plant_->mutable_X(0, 0) = (left_start_pos + right_start_pos) / 2.0;
    arm_plant_->mutable_X(1, 0) = 0.0;
    arm_plant_->mutable_X(2, 0) = (left_start_pos - right_start_pos) / 2.0;
    arm_plant_->mutable_X(3, 0) = 0.0;

    left_arm_pot_encoder_.Initialize(left_start_pos, pot_noise_stddev);
    right_arm_pot_encoder_.Initialize(right_start_pos, pot_noise_stddev);
    arm_initial_difference_ = left_start_pos - right_start_pos;
  }

  // Changes the left-right calculations in the limit checks to measure absolute
  // differences instead of differences relative to the starting offset.
  void ErrorOnAbsoluteDifference() {
    arm_initial_difference_ = 0.0;
    elevator_initial_difference_ = 0.0;
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<FridgeQueue::Position> position =
        fridge_queue_.position.MakeMessage();

    left_arm_pot_encoder_.GetSensorValues(&position->arm.left);
    right_arm_pot_encoder_.GetSensorValues(&position->arm.right);
    left_elevator_pot_encoder_.GetSensorValues(&position->elevator.left);
    right_elevator_pot_encoder_.GetSensorValues(&position->elevator.right);

    position.Send();
  }

  // Sets the difference between the commanded and applied power for the arm.
  // This lets us test that the integrator for the arm works.
  void set_arm_power_error(double arm_power_error) {
    arm_power_error_ = arm_power_error;
  }
  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(fridge_queue_.output.FetchLatest());

    ::Eigen::Matrix<double, 2, 1> arm_U;
    ::Eigen::Matrix<double, 2, 1> elevator_U;
    // Feed voltages into physics simulation.
    if (arm_power_error_ != 0.0) {
      arm_U << ::aos::Clip(
          fridge_queue_.output->left_arm + arm_power_error_, -12, 12),
          ::aos::Clip(fridge_queue_.output->right_arm + arm_power_error_, -12,
                      12);
    } else {
      arm_U << fridge_queue_.output->left_arm,
          fridge_queue_.output->right_arm;
    }
    elevator_U << fridge_queue_.output->left_elevator,
        fridge_queue_.output->right_elevator;

    // Use the plant to generate the next physical state given the voltages to
    // the motors.
    arm_plant_->Update(arm_U);
    elevator_plant_->Update(elevator_U);

    const double left_arm_angle = arm_plant_->Y(0, 0);
    const double right_arm_angle = arm_plant_->Y(1, 0);
    const double left_elevator_height = elevator_plant_->Y(0, 0);
    const double right_elevator_height = elevator_plant_->Y(1, 0);

    // TODO (phil) Do some sanity tests on the arm angles and the elevator
    // heights. For example, we need to make sure that both sides are within a
    // certain distance of each other and they haven't crashed into the top or
    // the bottom.

    // Use the physical state to simulate sensor readings.
    left_arm_pot_encoder_.MoveTo(left_arm_angle);
    right_arm_pot_encoder_.MoveTo(right_arm_angle);
    left_elevator_pot_encoder_.MoveTo(left_elevator_height);
    right_elevator_pot_encoder_.MoveTo(right_elevator_height);

    // Verify that the arm and elevator sides don't change much from their
    // initial difference.  Use the initial difference instead of the absolute
    // difference to handle starting too far apart to test e-stopping.
    EXPECT_NEAR(left_arm_angle - right_arm_angle, arm_initial_difference_,
                5.0 * M_PI / 180.0);
    EXPECT_NEAR(left_elevator_height - right_elevator_height,
                elevator_initial_difference_, 0.0254);

    // Validate that the arm is within range.
    EXPECT_GE(left_arm_angle,
              constants::GetValues().fridge.arm.lower_hard_limit);
    EXPECT_GE(right_arm_angle,
              constants::GetValues().fridge.arm.lower_hard_limit);
    EXPECT_LE(left_arm_angle,
              constants::GetValues().fridge.arm.upper_hard_limit);
    EXPECT_LE(right_arm_angle,
              constants::GetValues().fridge.arm.upper_hard_limit);

    // Validate that the elevator is within range.
    EXPECT_GE(left_elevator_height,
              constants::GetValues().fridge.elevator.lower_hard_limit);
    EXPECT_GE(right_elevator_height,
              constants::GetValues().fridge.elevator.lower_hard_limit);
    EXPECT_LE(left_elevator_height,
              constants::GetValues().fridge.elevator.upper_hard_limit);
    EXPECT_LE(right_elevator_height,
              constants::GetValues().fridge.elevator.upper_hard_limit);
  }

 private:
  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> arm_plant_;
  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> elevator_plant_;

  ::frc971::control_loops::PositionSensorSimulator left_arm_pot_encoder_;
  ::frc971::control_loops::PositionSensorSimulator right_arm_pot_encoder_;
  ::frc971::control_loops::PositionSensorSimulator left_elevator_pot_encoder_;
  ::frc971::control_loops::PositionSensorSimulator right_elevator_pot_encoder_;

  FridgeQueue fridge_queue_;

  double elevator_initial_difference_ = 0.0;
  double arm_initial_difference_ = 0.0;
  double arm_power_error_ = 0.0;
};

class FridgeTest : public ::aos::testing::ControlLoopTest {
 protected:
  FridgeTest()
      : fridge_queue_(".y2015.control_loops.fridge.fridge_queue", 0xe4e05855,
                      ".y2015.control_loops.fridge.fridge_queue.goal",
                      ".y2015.control_loops.fridge.fridge_queue.position",
                      ".y2015.control_loops.fridge.fridge_queue.output",
                      ".y2015.control_loops.fridge.fridge_queue.status"),
        fridge_(&fridge_queue_),
        fridge_plant_(),
        kinematics_(constants::GetValues().fridge.arm_length,
                    constants::GetValues().fridge.elevator.upper_limit,
                    constants::GetValues().fridge.elevator.lower_limit,
                    constants::GetValues().fridge.arm.upper_limit,
                    constants::GetValues().fridge.arm.lower_limit) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    fridge_queue_.goal.FetchLatest();
    fridge_queue_.status.FetchLatest();
    EXPECT_TRUE(fridge_queue_.goal.get() != nullptr);
    EXPECT_TRUE(fridge_queue_.status.get() != nullptr);
    if (fridge_queue_.goal->profiling_type == 0) {
      EXPECT_NEAR(fridge_queue_.goal->angle, fridge_queue_.status->angle,
                  0.001);
      EXPECT_NEAR(fridge_queue_.goal->height, fridge_queue_.status->height,
                  0.001);
    } else if (fridge_queue_.goal->profiling_type == 1) {
      aos::util::ElevatorArmKinematics::KinematicResult x_y_status;
      kinematics_.ForwardKinematic(fridge_queue_.status->height,
                                   fridge_queue_.status->angle, 0.0, 0.0, &x_y_status);
      EXPECT_NEAR(fridge_queue_.goal->x, x_y_status.fridge_x, 0.001);
      EXPECT_NEAR(fridge_queue_.goal->y, x_y_status.fridge_h, 0.001);
    } else {
      // Unhandled profiling type.
      EXPECT_TRUE(false);
    }
  }

  // Runs one iteration of the whole simulation and checks that separation
  // remains reasonable.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();

    TickTime();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const monotonic_clock::duration run_for,
                  bool enabled = true) {
    const auto start_time = monotonic_clock::now();
    while (monotonic_clock::now() < start_time + run_for) {
      RunIteration(enabled);
    }
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  FridgeQueue fridge_queue_;

  // Create a control loop and simulation.
  Fridge fridge_;
  FridgeSimulation fridge_plant_;

  aos::util::ElevatorArmKinematics kinematics_;
};

// Tests that the loop does nothing when the goal is zero.
TEST_F(FridgeTest, DoesNothing) {
  // Set the goals to the starting values. This should theoretically guarantee
  // that the controller does nothing.
  const auto &values = constants::GetValues();
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
                  .angle(0.0)
                  .height(values.fridge.elevator.lower_limit)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .max_angular_velocity(20)
                  .max_angular_acceleration(20)
                  .Send());

  // Run a few iterations.
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that the loop can reach a goal.
TEST_F(FridgeTest, ReachesXYGoal) {
  // Set a reasonable goal.
  const auto &values = constants::GetValues();
  const double soft_limit = values.fridge.elevator.lower_limit;
  const double height = soft_limit + 0.4;
  const double angle = M_PI / 6.0;

  aos::util::ElevatorArmKinematics::KinematicResult x_y_goals;
  kinematics_.ForwardKinematic(height, angle, 0.0, 0.0, &x_y_goals);

  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
                  .profiling_type(1)
                  .x(x_y_goals.fridge_x)
                  .y(x_y_goals.fridge_h)
                  .max_x_velocity(20)
                  .max_y_velocity(20)
                  .max_x_acceleration(20)
                  .max_y_acceleration(20)
                  .Send());

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that the loop can reach a goal.
TEST_F(FridgeTest, ReachesGoal) {
  // Set a reasonable goal.
  const auto &values = constants::GetValues();
  const double soft_limit = values.fridge.elevator.lower_limit;
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
                  .angle(M_PI / 4.0)
                  .height(soft_limit + 0.5)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .max_angular_velocity(20)
                  .max_angular_acceleration(20)
                  .Send());

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that the loop doesn't try and go beyond the physical range of the
// mechanisms.
TEST_F(FridgeTest, RespectsRange) {
  // Put the arm up to get it out of the way.
  // We're going to send the elevator to -5, which should be significantly too
  // low.
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
                  .angle(M_PI)
                  .height(-5.0)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .max_angular_velocity(20)
                  .max_angular_acceleration(20)
                  .Send());

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  fridge_queue_.status.FetchLatest();
  const auto &values = constants::GetValues();
  EXPECT_NEAR(values.fridge.elevator.lower_limit, fridge_queue_.status->height,
              0.001);
  EXPECT_NEAR(values.fridge.arm.upper_limit, fridge_queue_.status->angle,
              0.001);

  // Put the arm down to get it out of the way.
  // We're going to give the elevator some ridiculously high goal.
  ASSERT_TRUE(fridge_queue_.goal.MakeWithBuilder()
                  .angle(-M_PI)
                  .height(50.0)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .max_angular_velocity(20)
                  .max_angular_acceleration(20)
                  .Send());

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  fridge_queue_.status.FetchLatest();
  EXPECT_NEAR(values.fridge.elevator.upper_limit, fridge_queue_.status->height,
              0.001);
  EXPECT_NEAR(values.fridge.arm.lower_limit, fridge_queue_.status->angle,
              0.001);
}

// Tests that the loop zeroes when run for a while.
TEST_F(FridgeTest, ZeroTest) {
  fridge_queue_.goal.MakeWithBuilder()
      .angle(0.0)
      .height(0.5)
      .max_velocity(20)
      .max_acceleration(20)
      .max_angular_velocity(20)
      .max_angular_acceleration(20)
      .Send();
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that starting at the lower hardstops doesn't cause an abort.
TEST_F(FridgeTest, LowerHardstopStartup) {
  fridge_plant_.InitializeElevatorPosition(
      constants::GetValues().fridge.elevator.lower_hard_limit,
      constants::GetValues().fridge.elevator.lower_hard_limit);
  fridge_plant_.InitializeArmPosition(
      constants::GetValues().fridge.arm.lower_hard_limit,
      constants::GetValues().fridge.arm.lower_hard_limit);
  fridge_queue_.goal.MakeWithBuilder().angle(0.0).height(0.4).Send();
  // We have to wait for it to put the elevator in a safe position as well.
  RunForTime(chrono::milliseconds(8000));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(FridgeTest, UpperHardstopStartup) {
  fridge_plant_.InitializeElevatorPosition(
      constants::GetValues().fridge.elevator.upper_hard_limit,
      constants::GetValues().fridge.elevator.upper_hard_limit);
  fridge_plant_.InitializeArmPosition(
      constants::GetValues().fridge.arm.upper_hard_limit,
      constants::GetValues().fridge.arm.upper_hard_limit);
  fridge_queue_.goal.MakeWithBuilder().angle(0.0).height(0.4).Send();
  RunForTime(chrono::milliseconds(5000));

  VerifyNearGoal();
}

// Tests that starting with an initial arm difference triggers an ESTOP.
TEST_F(FridgeTest, ArmFarApartEstop) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.0).height(0.4).Send();

  do {
    fridge_plant_.InitializeArmPosition(
        constants::GetValues().fridge.arm.lower_limit,
        constants::GetValues().fridge.arm.lower_limit + 0.2);
    SendMessages(true);
    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();
    TickTime();
  } while (fridge_.state() == Fridge::INITIALIZING);

  EXPECT_EQ(Fridge::ZEROING_ELEVATOR, fridge_.state());

  // TODO(austin): We should estop earlier once Phil's code to detect issues
  // before the index pulse is merged.
  while (fridge_.state() != Fridge::RUNNING &&
         fridge_.state() != Fridge::ESTOP) {
    SendMessages(true);
    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();
    TickTime();
  }

  EXPECT_EQ(Fridge::ESTOP, fridge_.state());
}

// Tests that starting with an initial elevator difference triggers an ESTOP.
TEST_F(FridgeTest, ElevatorFarApartEstop) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.0).height(0.4).Send();

  do {
    fridge_plant_.InitializeElevatorPosition(
        constants::GetValues().fridge.elevator.lower_limit,
        constants::GetValues().fridge.elevator.lower_limit + 0.1);
    SendMessages(true);
    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();
    TickTime();
  } while (fridge_.state() == Fridge::INITIALIZING);

  EXPECT_EQ(Fridge::ZEROING_ELEVATOR, fridge_.state());

  // TODO(austin): We should estop earlier once Phil's code to detect issues
  // before the index pulse is merged.
  while (fridge_.state() != Fridge::RUNNING &&
         fridge_.state() != Fridge::ESTOP) {
    SendMessages(true);
    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();
    TickTime();
  }

  EXPECT_EQ(Fridge::ESTOP, fridge_.state());
}

// Tests that starting with an initial elevator difference converges back to 0
// error when zeroed.
TEST_F(FridgeTest, ElevatorFixError) {
  fridge_queue_.goal.MakeWithBuilder()
      .angle(0.0)
      .height(0.2)
      .max_velocity(20)
      .max_acceleration(20)
      .Send();

  do {
    fridge_plant_.InitializeElevatorPosition(
        constants::GetValues().fridge.elevator.lower_limit,
        constants::GetValues().fridge.elevator.lower_limit + 0.01);
    fridge_plant_.ErrorOnAbsoluteDifference();
    SendMessages(true);
    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();
    TickTime();
  } while (fridge_.state() == Fridge::INITIALIZING);

  EXPECT_EQ(Fridge::ZEROING_ELEVATOR, fridge_.state());

  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that starting with an initial arm difference converges back to 0
// error when zeroed.
TEST_F(FridgeTest, ArmFixError) {
  fridge_queue_.goal.MakeWithBuilder()
      .angle(0.0)
      .height(0.2)
      .max_angular_velocity(20)
      .max_angular_acceleration(20)
      .Send();

  do {
    fridge_plant_.InitializeArmPosition(0.0, 0.02);
    fridge_plant_.ErrorOnAbsoluteDifference();
    SendMessages(true);
    fridge_plant_.SendPositionMessage();
    fridge_.Iterate();
    fridge_plant_.Simulate();
    TickTime();
  } while (fridge_.state() == Fridge::INITIALIZING);

  EXPECT_EQ(Fridge::ZEROING_ELEVATOR, fridge_.state());

  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(FridgeTest, ResetTest) {
  fridge_plant_.InitializeElevatorPosition(
      constants::GetValues().fridge.elevator.upper_hard_limit,
      constants::GetValues().fridge.elevator.upper_hard_limit);
  fridge_plant_.InitializeArmPosition(
      constants::GetValues().fridge.arm.upper_hard_limit,
      constants::GetValues().fridge.arm.upper_hard_limit);
  fridge_queue_.goal.MakeWithBuilder().angle(0.03).height(0.45).Send();
  RunForTime(chrono::milliseconds(5000));

  EXPECT_EQ(Fridge::RUNNING, fridge_.state());
  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));
  EXPECT_NE(Fridge::RUNNING, fridge_.state());
  RunForTime(chrono::milliseconds(6000));
  EXPECT_EQ(Fridge::RUNNING, fridge_.state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(FridgeTest, DisabledGoalTest) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.03).height(0.45).Send();

  RunForTime(chrono::milliseconds(100), false);
  EXPECT_EQ(0.0, fridge_.elevator_goal_);
  EXPECT_EQ(0.0, fridge_.arm_goal_);

  // Now make sure they move correctly
  RunForTime(chrono::milliseconds(4000), true);
  EXPECT_NE(0.0, fridge_.elevator_goal_);
  EXPECT_NE(0.0, fridge_.arm_goal_);
}

// Tests that the elevator zeroing goals don't wind up too far.
TEST_F(FridgeTest, ElevatorGoalPositiveWindupTest) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.03).height(0.45).Send();

  while (fridge_.state() != Fridge::ZEROING_ELEVATOR) {
    RunIteration();
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));
  double orig_fridge_goal = fridge_.elevator_goal_;
  fridge_.elevator_goal_ += 100.0;

  RunIteration();
  EXPECT_NEAR(orig_fridge_goal, fridge_.elevator_goal_, 0.05);

  RunIteration();

  EXPECT_EQ(fridge_.elevator_loop_->U(), fridge_.elevator_loop_->U_uncapped());
}

// Tests that the arm zeroing goals don't wind up too far.
TEST_F(FridgeTest, ArmGoalPositiveWindupTest) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.03).height(0.45).Send();

  int i = 0;
  while (fridge_.state() != Fridge::ZEROING_ARM) {
    RunIteration();
    ++i;
    ASSERT_LE(i, 10000);
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));
  double orig_fridge_goal = fridge_.arm_goal_;
  fridge_.arm_goal_ += 100.0;

  RunIteration();
  EXPECT_NEAR(orig_fridge_goal, fridge_.arm_goal_, 0.05);

  RunIteration();

  EXPECT_EQ(fridge_.arm_loop_->U(), fridge_.arm_loop_->U_uncapped());
}

// Tests that the elevator zeroing goals don't wind up too far.
TEST_F(FridgeTest, ElevatorGoalNegativeWindupTest) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.03).height(0.45).Send();

  while (fridge_.state() != Fridge::ZEROING_ELEVATOR) {
    RunIteration();
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));
  double orig_fridge_goal = fridge_.elevator_goal_;
  fridge_.elevator_goal_ -= 100.0;

  RunIteration();
  EXPECT_NEAR(orig_fridge_goal, fridge_.elevator_goal_, 0.05);

  RunIteration();

  EXPECT_EQ(fridge_.elevator_loop_->U(), fridge_.elevator_loop_->U_uncapped());
}

// Tests that the arm zeroing goals don't wind up too far.
TEST_F(FridgeTest, ArmGoalNegativeWindupTest) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.03).height(0.45).Send();

  while (fridge_.state() != Fridge::ZEROING_ARM) {
    RunIteration();
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));
  double orig_fridge_goal = fridge_.arm_goal_;
  fridge_.arm_goal_ -= 100.0;

  RunIteration();
  EXPECT_NEAR(orig_fridge_goal, fridge_.arm_goal_, 0.05);

  RunIteration();

  EXPECT_EQ(fridge_.arm_loop_->U(), fridge_.arm_loop_->U_uncapped());
}

// Tests that the loop zeroes when run for a while.
TEST_F(FridgeTest, ZeroNoGoal) {
  RunForTime(chrono::seconds(5));

  EXPECT_EQ(Fridge::RUNNING, fridge_.state());
}

// Tests that if we start at the bottom, the elevator moves to a safe height
// before zeroing the arm.
TEST_F(FridgeTest, SafeArmZeroing) {
  auto &values = constants::GetValues();
  fridge_plant_.InitializeElevatorPosition(
      values.fridge.elevator.lower_hard_limit);
  fridge_plant_.InitializeArmPosition(M_PI / 4.0);

  const auto start_time = monotonic_clock::now();
  double last_arm_goal = fridge_.arm_goal_;
  while (monotonic_clock::now() < start_time + chrono::milliseconds(4000)) {
    RunIteration();

    if (fridge_.state() != Fridge::ZEROING_ELEVATOR) {
      // Wait until we are zeroing the elevator.
      continue;
    }

    fridge_queue_.status.FetchLatest();
    ASSERT_TRUE(fridge_queue_.status.get() != nullptr);
    if (fridge_queue_.status->height > values.fridge.arm_zeroing_height) {
      // We had better not be trying to zero the arm...
      EXPECT_EQ(last_arm_goal, fridge_.arm_goal_);
      last_arm_goal = fridge_.arm_goal_;
    }
  }
}

// Tests that the arm integrator works.
TEST_F(FridgeTest, ArmIntegratorTest) {
  fridge_plant_.InitializeArmPosition(
      (constants::GetValues().fridge.arm.lower_hard_limit +
       constants::GetValues().fridge.arm.lower_hard_limit) /
      2.0);
  fridge_plant_.set_arm_power_error(1.0);
  fridge_queue_.goal.MakeWithBuilder().angle(0.0).height(0.4).Send();

  RunForTime(chrono::milliseconds(8000));

  VerifyNearGoal();
}

// Phil:
// TODO(austin): Check that we e-stop if encoder index pulse is not n
// revolutions away from last one. (got extra counts from noise, etc).
// TODO(austin): Check that we e-stop if pot disagrees too much with encoder
// after we are zeroed.

}  // namespace testing
}  // namespace fridge
}  // namespace control_loops
}  // namespace y2015
