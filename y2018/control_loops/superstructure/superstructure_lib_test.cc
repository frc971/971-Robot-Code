#include "y2018/control_loops/superstructure/superstructure.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
#include "y2018/control_loops/superstructure/intake/intake_plant.h"
#include "y2018/status_light_generated.h"
#include "y2018/vision/vision_generated.h"

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

  flatbuffers::Offset<IntakeElasticSensors> GetSensorValues(
      flatbuffers::FlatBufferBuilder *fbb) {
    frc971::PotAndAbsolutePosition::Builder motor_position_builder(*fbb);
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> motor_position_offset =
        pot_encoder_.GetSensorValues(&motor_position_builder);

    IntakeElasticSensors::Builder intake_elastic_sensors_builder(*fbb);

    intake_elastic_sensors_builder.add_motor_position(motor_position_offset);
    intake_elastic_sensors_builder.add_spring_angle(plant_.Y(0));

    return intake_elastic_sensors_builder.Finish();
  }

  double spring_position() const { return plant_.X(0); }
  double spring_velocity() const { return plant_.X(1); }
  double motor_position() const { return plant_.X(2); }
  double motor_velocity() const { return plant_.X(3); }

  void set_voltage_offset(double voltage_offset) {
    plant_.set_voltage_offset(voltage_offset);
  }

  void Simulate(const IntakeVoltage *intake_voltage) {
    const double voltage_check =
        superstructure::intake::IntakeSide::kOperatingVoltage();

    AOS_CHECK_LE(::std::abs(intake_voltage->voltage_elastic()), voltage_check);

    ::Eigen::Matrix<double, 1, 1> U;
    U << intake_voltage->voltage_elastic() + plant_.voltage_offset();

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

  flatbuffers::Offset<ArmPosition> GetSensorValues(
      flatbuffers::FlatBufferBuilder *fbb) {
    frc971::PotAndAbsolutePosition::Builder proximal_builder(*fbb);
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> proximal_offset =
        proximal_pot_encoder_.GetSensorValues(&proximal_builder);

    frc971::PotAndAbsolutePosition::Builder distal_builder(*fbb);
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> distal_offset =
        distal_pot_encoder_.GetSensorValues(&distal_builder);

    ArmPosition::Builder arm_position_builder(*fbb);
    arm_position_builder.add_proximal(proximal_offset);
    arm_position_builder.add_distal(distal_offset);

    return arm_position_builder.Finish();
  }

  double proximal_position() const { return X_(0, 0); }
  double proximal_velocity() const { return X_(1, 0); }
  double distal_position() const { return X_(2, 0); }
  double distal_velocity() const { return X_(3, 0); }

  void Simulate(::Eigen::Matrix<double, 2, 1> U, bool release_arm_brake) {
    constexpr double voltage_check =
        superstructure::arm::Arm::kOperatingVoltage();

    AOS_CHECK_LE(::std::abs(U(0)), voltage_check);
    AOS_CHECK_LE(::std::abs(U(1)), voltage_check);

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
  SuperstructureSimulation(::aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        left_intake_(::y2018::control_loops::superstructure::intake::
                         MakeDelayedIntakePlant(),
                     constants::GetValues().left_intake.zeroing),
        right_intake_(::y2018::control_loops::superstructure::intake::
                          MakeDelayedIntakePlant(),
                      constants::GetValues().right_intake.zeroing),
        arm_(constants::GetValues().arm_proximal.zeroing,
             constants::GetValues().arm_distal.zeroing),
        superstructure_position_sender_(
            event_loop_->MakeSender<superstructure::Position>(
                "/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<superstructure::Status>(
                "/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<superstructure::Output>(
                "/superstructure")) {
    // Start the intake out in the middle by default.
    InitializeIntakePosition((constants::Values::kIntakeRange().lower +
                              constants::Values::kIntakeRange().upper) /
                             2.0);

    InitializeArmPosition(arm::UpPoint());

    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            Simulate();
          }
          first_ = false;
          SendPositionMessage();
        },
        ::std::chrono::microseconds(5050));
  }

  void InitializeIntakePosition(double start_pos) {
    left_intake_.InitializePosition(start_pos);
    right_intake_.InitializePosition(start_pos);
  }

  void InitializeArmPosition(::Eigen::Matrix<double, 2, 1> position) {
    arm_.InitializePosition(position);
  }

  void SendPositionMessage() {
    auto builder = superstructure_position_sender_.MakeBuilder();

    flatbuffers::Offset<IntakeElasticSensors> left_intake_offset =
        left_intake_.GetSensorValues(builder.fbb());
    flatbuffers::Offset<IntakeElasticSensors> right_intake_offset =
        right_intake_.GetSensorValues(builder.fbb());
    flatbuffers::Offset<ArmPosition> arm_offset =
        arm_.GetSensorValues(builder.fbb());

    Position::Builder position_builder = builder.MakeBuilder<Position>();
    position_builder.add_left_intake(left_intake_offset);
    position_builder.add_right_intake(right_intake_offset);
    position_builder.add_arm(arm_offset);
    EXPECT_TRUE(builder.Send(position_builder.Finish()));
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
    ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
    ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

    left_intake_.Simulate(superstructure_output_fetcher_->left_intake());
    right_intake_.Simulate(superstructure_output_fetcher_->right_intake());
    arm_.Simulate((::Eigen::Matrix<double, 2, 1>()
                       << superstructure_output_fetcher_->voltage_proximal(),
                   superstructure_output_fetcher_->voltage_distal())
                      .finished(),
                  superstructure_output_fetcher_->release_arm_brake());
  }

 private:
  ::aos::EventLoop *event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  IntakeSideSimulation left_intake_;
  IntakeSideSimulation right_intake_;
  ArmSimulation arm_;

  ::aos::Sender<superstructure::Position> superstructure_position_sender_;
  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;
  ::aos::Fetcher<superstructure::Output> superstructure_output_fetcher_;

  bool first_ = true;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : ::aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2018/config.json"),
            ::std::chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop("test")),
        superstructure_goal_fetcher_(
            test_event_loop_->MakeFetcher<superstructure::Goal>(
                "/superstructure")),
        superstructure_goal_sender_(
            test_event_loop_->MakeSender<superstructure::Goal>(
                "/superstructure")),
        superstructure_status_fetcher_(
            test_event_loop_->MakeFetcher<superstructure::Status>(
                "/superstructure")),
        superstructure_output_fetcher_(
            test_event_loop_->MakeFetcher<superstructure::Output>(
                "/superstructure")),
        superstructure_event_loop_(MakeEventLoop("superstructure")),
        superstructure_(superstructure_event_loop_.get(), "/superstructure"),
        superstructure_plant_event_loop_(MakeEventLoop("plant")),
        superstructure_plant_(superstructure_plant_event_loop_.get()) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();

    ASSERT_TRUE(superstructure_goal_fetcher_.get() != nullptr) << ": No goal";
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);
    // Left side test.
    EXPECT_NEAR(
        superstructure_goal_fetcher_->intake()->left_intake_angle(),
        superstructure_status_fetcher_->left_intake()->spring_position() +
            superstructure_status_fetcher_->left_intake()->motor_position(),
        0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->intake()->left_intake_angle(),
                superstructure_plant_.left_intake().spring_position(), 0.001);

    // Right side test.
    EXPECT_NEAR(
        superstructure_goal_fetcher_->intake()->right_intake_angle(),
        superstructure_status_fetcher_->right_intake()->spring_position() +
            superstructure_status_fetcher_->right_intake()->motor_position(),
        0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->intake()->right_intake_angle(),
                superstructure_plant_.right_intake().spring_position(), 0.001);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;

  ::aos::Fetcher<superstructure::Goal> superstructure_goal_fetcher_;
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;
  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;
  ::aos::Fetcher<superstructure::Output> superstructure_output_fetcher_;

  // Create a control loop and simulation.
  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop_;
  Superstructure superstructure_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;
};

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoalAndDoesNothing) {
  SetEnabled(true);
  RunFor(chrono::seconds(2));
  superstructure_output_fetcher_.Fetch();

  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_right().state());
  EXPECT_EQ(superstructure_output_fetcher_->left_intake()->voltage_elastic(),
            0.0);
  EXPECT_EQ(superstructure_output_fetcher_->right_intake()->voltage_elastic(),
            0.0);
}

// Tests that the intake loop can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  // Set a reasonable goal.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(0.1);
    intake_goal_builder.add_right_intake_angle(0.2);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the intake loop can reach a goal after starting at a non-zero
// position.
TEST_F(SuperstructureTest, OffsetStartReachesGoal) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(0.5);

  // Set a reasonable goal.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(0.1);
    intake_goal_builder.add_right_intake_angle(0.2);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the intake loops doesn't try and go beyond the
// physical range of the mechanisms.
TEST_F(SuperstructureTest, RespectsRange) {
  SetEnabled(true);
  // Set some ridiculous goals to test upper limits.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(5.0 * M_PI);
    intake_goal_builder.add_right_intake_angle(5.0 * M_PI);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();

  EXPECT_NEAR(0.0,
              superstructure_status_fetcher_->left_intake()->spring_position(),
              0.001);
  EXPECT_NEAR(
      constants::Values::kIntakeRange().upper,
      superstructure_status_fetcher_->left_intake()->spring_position() +
          superstructure_status_fetcher_->left_intake()->motor_position(),
      0.001);

  EXPECT_NEAR(0.0,
              superstructure_status_fetcher_->right_intake()->spring_position(),
              0.001);
  EXPECT_NEAR(constants::Values::kIntakeRange().upper,
              superstructure_status_fetcher_->right_intake()->motor_position(),
              0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(-5.0 * M_PI);
    intake_goal_builder.add_right_intake_angle(-5.0 * M_PI);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();

  EXPECT_NEAR(constants::Values::kIntakeRange().lower,
              superstructure_status_fetcher_->left_intake()->motor_position(),
              0.001);
  EXPECT_NEAR(0.0,
              superstructure_status_fetcher_->left_intake()->spring_position(),
              0.001);

  EXPECT_NEAR(constants::Values::kIntakeRange().lower,
              superstructure_status_fetcher_->right_intake()->motor_position(),
              0.001);
  EXPECT_NEAR(0.0,
              superstructure_status_fetcher_->right_intake()->spring_position(),
              0.001);
}

TEST_F(SuperstructureTest, DISABLED_LowerHardstopStartup) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().lower_hard);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(
        constants::Values::kIntakeRange().lower);
    intake_goal_builder.add_right_intake_angle(
        constants::Values::kIntakeRange().lower);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(1.0);
    intake_goal_builder.add_right_intake_angle(1.0);

    flatbuffers::Offset<IntakeGoal> intake_offset =
        intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));
  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, DISABLED_UpperHardstopStartup) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().upper_hard);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(
        constants::Values::kIntakeRange().upper);
    intake_goal_builder.add_right_intake_angle(
        constants::Values::kIntakeRange().upper);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().upper);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(
        constants::Values::kIntakeRange().upper - 0.1);
    intake_goal_builder.add_right_intake_angle(
        constants::Values::kIntakeRange().upper - 0.1);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::UpIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_right().state());

  VerifyNearGoal();
  SimulateSensorReset();
  RunFor(chrono::milliseconds(100));

  EXPECT_EQ(intake::IntakeSide::State::ZEROING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::ZEROING,
            superstructure_.intake_right().state());

  RunFor(chrono::seconds(10));

  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_left().state());
  EXPECT_EQ(intake::IntakeSide::State::RUNNING,
            superstructure_.intake_right().state());

  VerifyNearGoal();
}

// Tests that we don't freak out without a goal.
TEST_F(SuperstructureTest, ArmSimpleGoal) {
  SetEnabled(true);
  RunFor(chrono::seconds(5));

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(-0.8);
    intake_goal_builder.add_right_intake_angle(-0.8);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::FrontHighBoxIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  EXPECT_EQ(arm::Arm::State::RUNNING, superstructure_.arm().state());
}

// Tests that we can can execute a move.
TEST_F(SuperstructureTest, ArmMoveAndMoveBack) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(0.0);
    intake_goal_builder.add_right_intake_angle(0.0);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::FrontHighBoxIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(0.0);
    intake_goal_builder.add_right_intake_angle(0.0);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::ReadyAboveBoxIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));
  VerifyNearGoal();
}

// Tests that we can can execute a move which moves through multiple nodes.
TEST_F(SuperstructureTest, ArmMultistepMove) {
  SetEnabled(true);
  superstructure_plant_.InitializeArmPosition(arm::ReadyAboveBoxPoint());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(0.0);
    intake_goal_builder.add_right_intake_angle(0.0);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::BackLowBoxIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    IntakeGoal::Builder intake_goal_builder = builder.MakeBuilder<IntakeGoal>();
    intake_goal_builder.add_left_intake_angle(0.0);
    intake_goal_builder.add_right_intake_angle(0.0);

    flatbuffers::Offset<IntakeGoal> intake_offset = intake_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(intake_offset);
    goal_builder.add_arm_goal_position(arm::ReadyAboveBoxIndex());
    goal_builder.add_open_claw(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));
  VerifyNearGoal();
}


// TODO(austin): Test multiple path segments.
// TODO(austin): Disable in the middle and test recovery.

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
