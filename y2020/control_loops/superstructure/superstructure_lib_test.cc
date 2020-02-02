#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2020/constants.h"
#include "y2020/control_loops/superstructure/hood/hood_plant.h"
#include "y2020/control_loops/superstructure/intake/intake_plant.h"
#include "y2020/control_loops/superstructure/superstructure.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace testing {

namespace {
constexpr double kNoiseScalar = 0.01;
}  // namespace

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;
using ::frc971::CreateProfileParameters;
using ::frc971::control_loops::CappedTestPlant;
using ::frc971::control_loops::
    CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using ::frc971::control_loops::PositionSensorSimulator;
using ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
typedef Superstructure::AbsoluteEncoderSubsystem AbsoluteEncoderSubsystem;
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;

// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")),

        hood_plant_(new CappedTestPlant(hood::MakeHoodPlant())),
        hood_encoder_(constants::GetValues()
                          .hood.zeroing_constants.one_revolution_distance),
        intake_plant_(new CappedTestPlant(intake::MakeIntakePlant())),
        intake_encoder_(constants::GetValues()
                            .intake.zeroing_constants.one_revolution_distance),
        turret_plant_(new CappedTestPlant(turret::MakeTurretPlant())),
        turret_encoder_(constants::GetValues()
                            .turret.subsystem_params.zeroing_constants
                            .one_revolution_distance) {
    InitializeHoodPosition(constants::Values::kHoodRange().upper);
    InitializeIntakePosition(constants::Values::kIntakeRange().upper);
    InitializeTurretPosition(constants::Values::kTurretRange().middle());

    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            Simulate();
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
  }

  void InitializeHoodPosition(double start_pos) {
    hood_plant_->mutable_X(0, 0) = start_pos;
    hood_plant_->mutable_X(1, 0) = 0.0;

    hood_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .hood.zeroing_constants.measured_absolute_position);
  }

  void InitializeIntakePosition(double start_pos) {
    intake_plant_->mutable_X(0, 0) = start_pos;
    intake_plant_->mutable_X(1, 0) = 0.0;

    intake_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .intake.zeroing_constants.measured_absolute_position);
  }

  void InitializeTurretPosition(double start_pos) {
    turret_plant_->mutable_X(0, 0) = start_pos;
    turret_plant_->mutable_X(1, 0) = 0.0;

    turret_encoder_.Initialize(start_pos, kNoiseScalar, 0.0,
                               constants::GetValues()
                                   .turret.subsystem_params.zeroing_constants
                                   .measured_absolute_position);
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::AbsolutePosition::Builder hood_builder =
        builder.MakeBuilder<frc971::AbsolutePosition>();
    flatbuffers::Offset<frc971::AbsolutePosition> hood_offset =
        hood_encoder_.GetSensorValues(&hood_builder);

    frc971::AbsolutePosition::Builder intake_builder =
        builder.MakeBuilder<frc971::AbsolutePosition>();
    flatbuffers::Offset<frc971::AbsolutePosition> intake_offset =
        intake_encoder_.GetSensorValues(&intake_builder);

    frc971::PotAndAbsolutePosition::Builder turret_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> turret_offset =
        turret_encoder_.GetSensorValues(&turret_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_hood(hood_offset);
    position_builder.add_intake_joint(intake_offset);
    position_builder.add_turret(turret_offset);

    builder.Send(position_builder.Finish());
  }

  double hood_position() const { return hood_plant_->X(0, 0); }
  double hood_velocity() const { return hood_plant_->X(1, 0); }

  double intake_position() const { return intake_plant_->X(0, 0); }
  double intake_velocity() const { return intake_plant_->X(1, 0); }

  double turret_position() const { return turret_plant_->X(0, 0); }
  double turret_velocity() const { return turret_plant_->X(1, 0); }

  // Simulates the superstructure for a single timestep.
  void Simulate() {
    const double last_hood_velocity = hood_velocity();
    const double last_intake_velocity = intake_velocity();
    const double last_turret_velocity = turret_velocity();

    EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
    EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

    const double voltage_check_hood =
        (static_cast<AbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->hood()->state()) ==
         AbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().hood.operating_voltage
            : constants::GetValues().hood.zeroing_voltage;

    EXPECT_NEAR(superstructure_output_fetcher_->hood_voltage(), 0.0,
                voltage_check_hood);

    const double voltage_check_intake =
        (static_cast<AbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->intake()->state()) ==
         AbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().intake.operating_voltage
            : constants::GetValues().intake.zeroing_voltage;

    EXPECT_NEAR(superstructure_output_fetcher_->intake_joint_voltage(), 0.0,
                voltage_check_intake);

    const double voltage_check_turret =
        (static_cast<PotAndAbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->turret()->state()) ==
         PotAndAbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().turret.subsystem_params.operating_voltage
            : constants::GetValues().turret.subsystem_params.zeroing_voltage;

    EXPECT_NEAR(superstructure_output_fetcher_->turret_voltage(), 0.0,
                voltage_check_turret);

    ::Eigen::Matrix<double, 1, 1> hood_U;
    hood_U << superstructure_output_fetcher_->hood_voltage() +
                  hood_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> intake_U;
    intake_U << superstructure_output_fetcher_->intake_joint_voltage() +
                    intake_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> turret_U;
    turret_U << superstructure_output_fetcher_->turret_voltage() +
                    turret_plant_->voltage_offset();

    hood_plant_->Update(hood_U);
    intake_plant_->Update(intake_U);
    turret_plant_->Update(turret_U);

    const double position_hood = hood_plant_->Y(0, 0);
    const double position_intake = intake_plant_->Y(0, 0);
    const double position_turret = turret_plant_->Y(0, 0);

    hood_encoder_.MoveTo(position_hood);
    intake_encoder_.MoveTo(position_intake);
    turret_encoder_.MoveTo(position_turret);

    EXPECT_GE(position_hood, constants::Values::kHoodRange().lower_hard);
    EXPECT_LE(position_hood, constants::Values::kHoodRange().upper_hard);

    EXPECT_GE(position_intake, constants::Values::kIntakeRange().lower_hard);
    EXPECT_LE(position_intake, constants::Values::kIntakeRange().upper_hard);

    EXPECT_GE(position_turret, constants::Values::kTurretRange().lower_hard);
    EXPECT_LE(position_turret, constants::Values::kTurretRange().upper_hard);

    const double loop_time = ::aos::time::DurationInSeconds(dt_);

    const double hood_acceleration =
        (hood_velocity() - last_hood_velocity) / loop_time;

    const double intake_acceleration =
        (intake_velocity() - last_intake_velocity) / loop_time;

    const double turret_acceleration =
        (turret_velocity() - last_turret_velocity) / loop_time;

    EXPECT_GE(peak_hood_acceleration_, hood_acceleration);
    EXPECT_LE(-peak_hood_acceleration_, hood_acceleration);
    EXPECT_GE(peak_hood_velocity_, hood_velocity());
    EXPECT_LE(-peak_hood_velocity_, hood_velocity());

    EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
    EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);
    EXPECT_GE(peak_intake_velocity_, intake_velocity());
    EXPECT_LE(-peak_intake_velocity_, intake_velocity());

    EXPECT_GE(peak_turret_acceleration_, turret_acceleration);
    EXPECT_LE(-peak_turret_acceleration_, turret_acceleration);
    EXPECT_GE(peak_turret_velocity_, turret_velocity());
    EXPECT_LE(-peak_turret_velocity_, turret_velocity());

    climber_voltage_ = superstructure_output_fetcher_->climber_voltage();
  }

  float climber_voltage() const { return climber_voltage_; }

  void set_peak_hood_acceleration(double value) {
    peak_hood_acceleration_ = value;
  }
  void set_peak_hood_velocity(double value) { peak_hood_velocity_ = value; }

  void set_peak_intake_acceleration(double value) {
    peak_intake_acceleration_ = value;
  }
  void set_peak_intake_velocity(double value) { peak_intake_velocity_ = value; }

  void set_peak_turret_acceleration(double value) {
    peak_turret_acceleration_ = value;
  }
  void set_peak_turret_velocity(double value) { peak_turret_velocity_ = value; }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool first_ = true;

  ::std::unique_ptr<CappedTestPlant> hood_plant_;
  PositionSensorSimulator hood_encoder_;

  ::std::unique_ptr<CappedTestPlant> intake_plant_;
  PositionSensorSimulator intake_encoder_;

  ::std::unique_ptr<CappedTestPlant> turret_plant_;
  PositionSensorSimulator turret_encoder_;

  // The acceleration limits to check for while moving.
  double peak_hood_acceleration_ = 1e10;
  double peak_intake_acceleration_ = 1e10;
  double peak_turret_acceleration_ = 1e10;

  // The velocity limits to check for while moving.
  double peak_hood_velocity_ = 1e10;
  double peak_intake_velocity_ = 1e10;
  double peak_turret_velocity_ = 1e10;

  float climber_voltage_ = 0.0f;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : ::aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2020/config.json"),
            chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop("test")),
        superstructure_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/superstructure")),
        superstructure_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/superstructure")),
        superstructure_status_fetcher_(
            test_event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            test_event_loop_->MakeFetcher<Output>("/superstructure")),
        superstructure_position_fetcher_(
            test_event_loop_->MakeFetcher<Position>("/superstructure")),
        superstructure_event_loop_(MakeEventLoop("superstructure")),
        superstructure_(superstructure_event_loop_.get()),
        superstructure_plant_event_loop_(MakeEventLoop("plant")),
        superstructure_plant_(superstructure_plant_event_loop_.get(), dt()) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();

    // Only check the goal if there is one.
    if (superstructure_goal_fetcher_->has_hood()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->hood()->unsafe_goal(),
                  superstructure_status_fetcher_->hood()->position(), 0.001);
    }

    if (superstructure_goal_fetcher_->has_intake()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->intake()->unsafe_goal(),
                  superstructure_status_fetcher_->intake()->position(), 0.001);
    }

    if (superstructure_goal_fetcher_->has_turret()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->turret()->unsafe_goal(),
                  superstructure_status_fetcher_->turret()->position(), 0.001);
    }
  }

  void CheckIfZeroed() {
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_status_fetcher_.get()->zeroed());
  }

  void WaitUntilZeroed() {
    int i = 0;
    do {
      i++;
      RunFor(dt());
      superstructure_status_fetcher_.Fetch();
      // 2 Seconds
      ASSERT_LE(i, 2.0 / ::aos::time::DurationInSeconds(dt()));

      // Since there is a delay when sending running, make sure we have a status
      // before checking it.
    } while (superstructure_status_fetcher_.get() == nullptr ||
             !superstructure_status_fetcher_.get()->zeroed());
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;

  // Create a control loop and simulation.
  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop_;
  Superstructure superstructure_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;
};

// Tests that the superstructure does nothing when the goal is to remain still.
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange().middle());
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().middle());

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().middle());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().middle());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().middle() + 1.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));
  VerifyNearGoal();

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
}

// Tests that loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  // Set a reasonable goal.

  superstructure_plant_.InitializeHoodPosition(0.7);
  superstructure_plant_.InitializeIntakePosition(0.7);

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.2,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.2,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().middle() + 1.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}
// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
TEST_F(SuperstructureTest, SaturationTest) {
  SetEnabled(true);
  // Zero it before we move.
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().middle() + 1.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(8));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().middle() + 1.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_peak_hood_velocity(23.0);
  superstructure_plant_.set_peak_hood_acceleration(0.2);

  superstructure_plant_.set_peak_intake_velocity(23.0);
  superstructure_plant_.set_peak_intake_acceleration(0.2);

  superstructure_plant_.set_peak_turret_velocity(23.0);
  superstructure_plant_.set_peak_turret_acceleration(0.2);

  // Intake needs over 8 seconds to reach the goal
  RunFor(chrono::seconds(9));
  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));
  EXPECT_EQ(AbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.hood().state());

  EXPECT_EQ(AbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.intake_joint().state());

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.turret().state());
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
}

// Tests that the climber passes through per the design.
TEST_F(SuperstructureTest, Climber) {
  SetEnabled(true);
  // Set a reasonable goal.

  superstructure_plant_.InitializeHoodPosition(0.7);
  superstructure_plant_.InitializeIntakePosition(0.7);

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_voltage(-10.0);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it time to stabilize.
  RunFor(chrono::seconds(1));

  // Can't go backwards.
  EXPECT_EQ(superstructure_plant_.climber_voltage(), 0.0);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_voltage(10.0);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(1));
  // But forwards works.
  EXPECT_EQ(superstructure_plant_.climber_voltage(), 10.0);

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
