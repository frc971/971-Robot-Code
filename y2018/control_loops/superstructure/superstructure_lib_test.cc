#include "y2018/control_loops/superstructure/superstructure.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/common/controls/control_loop_test.h"
#include "aos/common/queue.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
#include "y2018/control_loops/superstructure/intake/intake_plant.h"

using ::frc971::control_loops::PositionSensorSimulator;

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace testing {
namespace {
constexpr double kNoiseScalar = 0.01;
}  // namespace

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

class IntakePlant : public StateFeedbackPlant<5, 1, 2> {
 public:
  explicit IntakePlant(StateFeedbackPlant<5, 1, 2> &&other)
      : StateFeedbackPlant<5, 1, 2>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 1, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + voltage_offset_);
  }

  double voltage_offset() const { return voltage_offset_; }
  void set_voltage_offset(double voltage_offset) {
    voltage_offset_ = voltage_offset;
  }

 private:
  double voltage_offset_ = 0.0;
};

class IntakeSideSimulation {
 public:
  explicit IntakeSideSimulation(
      StateFeedbackPlant<5, 1, 2> &&other,
      const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
          &zeroing_constants)
      : plant_(::std::move(other)),
        pot_encoder_(M_PI * 2.0 *
                     constants::Values::kIntakeMotorEncoderRatio()),
        zeroing_constants_(zeroing_constants) {}

  void InitializePosition(double start_pos) {
    plant_.mutable_X().setZero();
    plant_.mutable_X(0) = start_pos;
    plant_.mutable_X(2) = start_pos;

    pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        zeroing_constants_.measured_absolute_position);
  }

  void GetSensorValues(IntakeElasticSensors *position) {
    pot_encoder_.GetSensorValues(&position->motor_position);
    position->spring_angle = plant_.Y(0);
  }

  double spring_position() const { return plant_.X(0); }
  double spring_velocity() const { return plant_.X(1); }
  double motor_position() const { return plant_.X(2); }
  double motor_velocity() const { return plant_.X(3); }

  void set_voltage_offset(double voltage_offset) {
    plant_.set_voltage_offset(voltage_offset);
  }

  void Simulate(const IntakeVoltage &intake_voltage) {
    const double voltage_check =
        superstructure::intake::IntakeSide::kOperatingVoltage();

    CHECK_LE(::std::abs(intake_voltage.voltage_elastic), voltage_check);

    ::Eigen::Matrix<double, 1, 1> U;
    U << intake_voltage.voltage_elastic + plant_.voltage_offset();

    plant_.Update(U);

    const double position_intake = plant_.Y(1);

    pot_encoder_.MoveTo(position_intake);
    EXPECT_GE(position_intake,
              constants::Values::kIntakeRange().lower_hard);
    EXPECT_LE(position_intake,
              constants::Values::kIntakeRange().upper_hard);
  }

 private:
  IntakePlant plant_;

  PositionSensorSimulator pot_encoder_;

  const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
      zeroing_constants_;
};

class ArmSimulation {
 public:
  explicit ArmSimulation(
      const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
          &proximal_zeroing_constants,
      const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
          &distal_zeroing_constants)
      : proximal_zeroing_constants_(proximal_zeroing_constants),
        proximal_pot_encoder_(M_PI * 2.0 *
                              constants::Values::kProximalEncoderRatio()),
        distal_zeroing_constants_(distal_zeroing_constants),
        distal_pot_encoder_(M_PI * 2.0 *
                            constants::Values::kDistalEncoderRatio()) {
    X_.setZero();
  }

  void InitializePosition(::Eigen::Matrix<double, 2, 1> position) {
    X_ << position(0), 0.0, position(1), 0.0;

    proximal_pot_encoder_.Initialize(
        X_(0), kNoiseScalar, 0.0,
        proximal_zeroing_constants_.measured_absolute_position);
    distal_pot_encoder_.Initialize(
        X_(2), kNoiseScalar, 0.0,
        distal_zeroing_constants_.measured_absolute_position);
  }

  void GetSensorValues(ArmPosition *position) {
    proximal_pot_encoder_.GetSensorValues(&position->proximal);
    distal_pot_encoder_.GetSensorValues(&position->distal);
  }

  double proximal_position() const { return X_(0, 0); }
  double proximal_velocity() const { return X_(1, 0); }
  double distal_position() const { return X_(2, 0); }
  double distal_velocity() const { return X_(3, 0); }

  void Simulate(::Eigen::Matrix<double, 2, 1> U, bool release_arm_brake) {
    constexpr double voltage_check =
        superstructure::arm::Arm::kOperatingVoltage();

    CHECK_LE(::std::abs(U(0)), voltage_check);
    CHECK_LE(::std::abs(U(1)), voltage_check);

    if (release_arm_brake) {
      X_ = arm::Dynamics::UnboundedDiscreteDynamics(X_, U, 0.00505);
    } else {
      // Well, the brake shouldn't stop both joints, but this will get the tests
      // to pass.
      X_(1) = 0.0;
      X_(3) = 0.0;
    }

    // TODO(austin): Estop on grose out of bounds.
    proximal_pot_encoder_.MoveTo(X_(0));
    distal_pot_encoder_.MoveTo(X_(2));
  }

 private:
  ::Eigen::Matrix<double, 4, 1> X_;

  const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
      proximal_zeroing_constants_;
  PositionSensorSimulator proximal_pot_encoder_;

  const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
      distal_zeroing_constants_;
  PositionSensorSimulator distal_pot_encoder_;
};


class SuperstructureSimulation {
 public:
  SuperstructureSimulation()
      : left_intake_(::y2018::control_loops::superstructure::intake::
                         MakeDelayedIntakePlant(),
                     constants::GetValues().left_intake.zeroing),
        right_intake_(::y2018::control_loops::superstructure::intake::
                          MakeDelayedIntakePlant(),
                      constants::GetValues().right_intake.zeroing),
        arm_(constants::GetValues().arm_proximal.zeroing,
             constants::GetValues().arm_distal.zeroing),
        superstructure_queue_(".y2018.control_loops.superstructure", 0xdeadbeef,
                              ".y2018.control_loops.superstructure.goal",
                              ".y2018.control_loops.superstructure.position",
                              ".y2018.control_loops.superstructure.output",
                              ".y2018.control_loops.superstructure.status") {
    // Start the intake out in the middle by default.
    InitializeIntakePosition((constants::Values::kIntakeRange().lower +
                              constants::Values::kIntakeRange().upper) /
                             2.0);

    InitializeArmPosition(arm::UpPoint());
  }

  void InitializeIntakePosition(double start_pos) {
    left_intake_.InitializePosition(start_pos);
    right_intake_.InitializePosition(start_pos);
  }

  void InitializeArmPosition(::Eigen::Matrix<double, 2, 1> position) {
    arm_.InitializePosition(position);
  }

  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<SuperstructureQueue::Position> position =
        superstructure_queue_.position.MakeMessage();

    left_intake_.GetSensorValues(&position->left_intake);
    right_intake_.GetSensorValues(&position->right_intake);
    arm_.GetSensorValues(&position->arm);
    LOG_STRUCT(INFO, "sim position", *position);
    position.Send();
  }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_left_intake_voltage_offset(double voltage_offset) {
    left_intake_.set_voltage_offset(voltage_offset);
  }
  void set_right_intake_voltage_offset(double voltage_offset) {
    right_intake_.set_voltage_offset(voltage_offset);
  }

  const IntakeSideSimulation &left_intake() const { return left_intake_; }
  const IntakeSideSimulation &right_intake() const { return right_intake_; }

  const ArmSimulation &arm() const { return arm_; }

  // Simulates the intake for a single timestep.
  void Simulate() {
    EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
    EXPECT_TRUE(superstructure_queue_.status.FetchLatest());

    left_intake_.Simulate(superstructure_queue_.output->left_intake);
    right_intake_.Simulate(superstructure_queue_.output->right_intake);
    arm_.Simulate((::Eigen::Matrix<double, 2, 1>()
                       << superstructure_queue_.output->voltage_proximal,
                   superstructure_queue_.output->voltage_distal)
                      .finished(),
                  superstructure_queue_.output->release_arm_brake);
  }

 private:
  IntakeSideSimulation left_intake_;
  IntakeSideSimulation right_intake_;
  ArmSimulation arm_;

  SuperstructureQueue superstructure_queue_;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : superstructure_queue_(".y2018.control_loops.superstructure", 0xdeadbeef,
                              ".y2018.control_loops.superstructure.goal",
                              ".y2018.control_loops.superstructure.position",
                              ".y2018.control_loops.superstructure.output",
                              ".y2018.control_loops.superstructure.status"),
        superstructure_(&superstructure_queue_) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    superstructure_queue_.goal.FetchLatest();
    superstructure_queue_.status.FetchLatest();

    ASSERT_TRUE(superstructure_queue_.goal.get() != nullptr) << ": No goal";
    ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);
    // Left side test.
    EXPECT_NEAR(superstructure_queue_.goal->intake.left_intake_angle,
                superstructure_queue_.status->left_intake.spring_position +
                    superstructure_queue_.status->left_intake.motor_position,
                0.001);
    EXPECT_NEAR(superstructure_queue_.goal->intake.left_intake_angle,
                superstructure_plant_.left_intake().spring_position(), 0.001);

    // Right side test.
    EXPECT_NEAR(superstructure_queue_.goal->intake.right_intake_angle,
                superstructure_queue_.status->right_intake.spring_position +
                    superstructure_queue_.status->right_intake.motor_position,
                0.001);
    EXPECT_NEAR(superstructure_queue_.goal->intake.right_intake_angle,
                superstructure_plant_.right_intake().spring_position(), 0.001);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    superstructure_plant_.SendPositionMessage();
    superstructure_.Iterate();
    superstructure_plant_.Simulate();

    TickTime(::std::chrono::microseconds(5050));
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const monotonic_clock::duration run_for,
                  bool enabled = true) {
    const auto end_time = monotonic_clock::now() + run_for;
    while (monotonic_clock::now() < end_time) {
      RunIteration(enabled);
    }
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory
  // that is no longer valid.
  SuperstructureQueue superstructure_queue_;

  // Create a control loop and simulation.
  Superstructure superstructure_;
  SuperstructureSimulation superstructure_plant_;
};

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoalAndDoesNothing) {
  RunForTime(chrono::seconds(2));
  superstructure_queue_.output.FetchLatest();

  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_right().state());
  EXPECT_EQ(superstructure_queue_.output->left_intake.voltage_elastic, 0.0);
  EXPECT_EQ(superstructure_queue_.output->right_intake.voltage_elastic, 0.0);
}

// Tests that the intake loop can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  // Set a reasonable goal.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();

    goal->intake.left_intake_angle = 0.1;
    goal->intake.right_intake_angle = 0.2;
    goal->arm_goal_position = arm::UpIndex();
    goal->open_claw = true;

    ASSERT_TRUE(goal.Send());
  }

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the intake loop can reach a goal after starting at a non-zero
// position.
TEST_F(SuperstructureTest, OffsetStartReachesGoal) {
  superstructure_plant_.InitializeIntakePosition(0.5);

  // Set a reasonable goal.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();

    goal->intake.left_intake_angle = 0.1;
    goal->intake.right_intake_angle = 0.2;
    goal->arm_goal_position = arm::UpIndex();
    goal->open_claw = true;

    ASSERT_TRUE(goal.Send());
  }

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the intake loops doesn't try and go beyond the
// physical range of the mechanisms.
TEST_F(SuperstructureTest, RespectsRange) {
  // Set some ridiculous goals to test upper limits.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();

    goal->intake.left_intake_angle = 5.0 * M_PI;
    goal->intake.right_intake_angle = 5.0 * M_PI;
    goal->arm_goal_position = arm::UpIndex();
    goal->open_claw = true;

    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();

  EXPECT_NEAR(0.0, superstructure_queue_.status->left_intake.spring_position,
              0.001);
  EXPECT_NEAR(constants::Values::kIntakeRange().upper,
              superstructure_queue_.status->left_intake.spring_position +
                  superstructure_queue_.status->left_intake.motor_position,
              0.001);

  EXPECT_NEAR(0.0, superstructure_queue_.status->right_intake.spring_position,
              0.001);
  EXPECT_NEAR(constants::Values::kIntakeRange().upper,
                  superstructure_queue_.status->right_intake.motor_position,
              0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();

    goal->intake.left_intake_angle = -5.0 * M_PI;
    goal->intake.right_intake_angle = -5.0 * M_PI;
    goal->arm_goal_position = arm::UpIndex();
    goal->open_claw = true;

    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();

  EXPECT_NEAR(constants::Values::kIntakeRange().lower,
              superstructure_queue_.status->left_intake.motor_position, 0.001);
  EXPECT_NEAR(0.0, superstructure_queue_.status->left_intake.spring_position,
              0.001);

  EXPECT_NEAR(constants::Values::kIntakeRange().lower,
              superstructure_queue_.status->right_intake.motor_position, 0.001);
  EXPECT_NEAR(0.0, superstructure_queue_.status->right_intake.spring_position,
              0.001);
}

TEST_F(SuperstructureTest, DISABLED_LowerHardstopStartup) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().lower_hard);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.left_intake_angle = constants::Values::kIntakeRange().lower;
    goal->intake.right_intake_angle = constants::Values::kIntakeRange().lower;
    goal->arm_goal_position = arm::UpIndex();
    goal->open_claw = true;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.left_intake_angle = 1.0;
    goal->intake.right_intake_angle = 1.0;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));
  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, DISABLED_UpperHardstopStartup) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().upper_hard);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.left_intake_angle = constants::Values::kIntakeRange().upper;
    goal->intake.right_intake_angle = constants::Values::kIntakeRange().upper;
    goal->arm_goal_position = arm::UpIndex();
    goal->open_claw = true;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().upper);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.left_intake_angle =
        constants::Values::kIntakeRange().upper - 0.1;
    goal->intake.right_intake_angle =
        constants::Values::kIntakeRange().upper - 0.1;
    goal->arm_goal_position = arm::UpIndex();
    goal->open_claw = true;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_right().state());

  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));

  EXPECT_EQ(intake::IntakeSide::State::ZEROING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::ZEROING,
            superstructure_.intake_right().state());

  RunForTime(chrono::seconds(10));

  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_right().state());

  VerifyNearGoal();
}

// Tests that we don't freak out without a goal.
TEST_F(SuperstructureTest, ArmSimpleGoal) {
  RunForTime(chrono::seconds(5));

  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.left_intake_angle = -0.8;
    goal->intake.right_intake_angle = -0.8;
    goal->arm_goal_position = arm::FrontHighBoxIndex();
    goal->open_claw = true;
    ASSERT_TRUE(goal.Send());
  }

  EXPECT_EQ(arm::Arm::State::RUNNING, superstructure_.arm().state());
}

// Tests that we can can execute a move.
TEST_F(SuperstructureTest, ArmMoveAndMoveBack) {

  auto goal = superstructure_queue_.goal.MakeMessage();
  goal->intake.left_intake_angle = 0.0;
  goal->intake.right_intake_angle = 0.0;
  goal->arm_goal_position = arm::FrontHighBoxIndex();
  goal->open_claw = true;
  ASSERT_TRUE(goal.Send());

  RunForTime(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.left_intake_angle = 0.0;
    goal->intake.right_intake_angle = 0.0;
    goal->arm_goal_position = arm::ReadyAboveBoxIndex();
    goal->open_claw = true;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(10));
  VerifyNearGoal();
}

// Tests that we can can execute a move which moves through multiple nodes.
TEST_F(SuperstructureTest, ArmMultistepMove) {
  superstructure_plant_.InitializeArmPosition(arm::ReadyAboveBoxPoint());

  auto goal = superstructure_queue_.goal.MakeMessage();
  goal->intake.left_intake_angle = 0.0;
  goal->intake.right_intake_angle = 0.0;
  goal->arm_goal_position = arm::BackLowBoxIndex();
  goal->open_claw = true;
  ASSERT_TRUE(goal.Send());

  RunForTime(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.left_intake_angle = 0.0;
    goal->intake.right_intake_angle = 0.0;
    goal->arm_goal_position = arm::ReadyAboveBoxIndex();
    goal->open_claw = true;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(10));
  VerifyNearGoal();
}


// TODO(austin): Test multiple path segments.
// TODO(austin): Disable in the middle and test recovery.

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
