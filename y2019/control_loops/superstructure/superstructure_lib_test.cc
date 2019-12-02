#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2019/constants.h"
#include "y2019/control_loops/superstructure/elevator/elevator_plant.h"
#include "y2019/control_loops/superstructure/intake/intake_plant.h"
#include "y2019/control_loops/superstructure/stilts/stilts_plant.h"
#include "y2019/control_loops/superstructure/superstructure.h"
#include "y2019/control_loops/superstructure/wrist/wrist_plant.h"

namespace y2019 {
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
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;
typedef Superstructure::AbsoluteEncoderSubsystem AbsoluteEncoderSubsystem;

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
        elevator_plant_(
            new CappedTestPlant(::y2019::control_loops::superstructure::
                                    elevator::MakeElevatorPlant())),
        elevator_pot_encoder_(M_PI * 2.0 *
                              constants::Values::kElevatorEncoderRatio()),

        wrist_plant_(new CappedTestPlant(
            ::y2019::control_loops::superstructure::wrist::MakeWristPlant())),
        wrist_pot_encoder_(M_PI * 2.0 *
                           constants::Values::kWristEncoderRatio()),

        intake_plant_(new CappedTestPlant(
            ::y2019::control_loops::superstructure::intake::MakeIntakePlant())),
        intake_pot_encoder_(M_PI * 2.0 *
                            constants::Values::kIntakeEncoderRatio()),

        stilts_plant_(new CappedTestPlant(
            ::y2019::control_loops::superstructure::stilts::MakeStiltsPlant())),
        stilts_pot_encoder_(M_PI * 2.0 *
                            constants::Values::kStiltsEncoderRatio()) {
    // Start the elevator out in the middle by default.
    InitializeElevatorPosition(constants::Values::kElevatorRange().upper);

    // Start the wrist out in the middle by default.
    InitializeWristPosition(constants::Values::kWristRange().upper);

    InitializeIntakePosition(constants::Values::kIntakeRange().upper);

    // Start the stilts out in the middle by default.
    InitializeStiltsPosition(constants::Values::kStiltsRange().lower);

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

  void InitializeElevatorPosition(double start_pos) {
    elevator_plant_->mutable_X(0, 0) = start_pos;
    elevator_plant_->mutable_X(1, 0) = 0.0;

    elevator_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .elevator.subsystem_params.zeroing_constants
            .measured_absolute_position);
  }

  void InitializeWristPosition(double start_pos) {
    wrist_plant_->mutable_X(0, 0) = start_pos;
    wrist_plant_->mutable_X(1, 0) = 0.0;
    wrist_pot_encoder_.Initialize(start_pos, kNoiseScalar, 0.0,
                                  constants::GetValues()
                                      .wrist.subsystem_params.zeroing_constants
                                      .measured_absolute_position);
  }

  void InitializeIntakePosition(double start_pos) {
    intake_plant_->mutable_X(0, 0) = start_pos;
    intake_plant_->mutable_X(1, 0) = 0.0;

    intake_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .intake.zeroing_constants.measured_absolute_position);
  }

  void InitializeStiltsPosition(double start_pos) {
    stilts_plant_->mutable_X(0, 0) = start_pos;
    stilts_plant_->mutable_X(1, 0) = 0.0;

    stilts_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .stilts.subsystem_params.zeroing_constants
            .measured_absolute_position);
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::PotAndAbsolutePosition::Builder elevator_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> elevator_offset =
        elevator_pot_encoder_.GetSensorValues(&elevator_builder);

    frc971::PotAndAbsolutePosition::Builder wrist_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> wrist_offset =
        wrist_pot_encoder_.GetSensorValues(&wrist_builder);

    frc971::AbsolutePosition::Builder intake_builder =
        builder.MakeBuilder<frc971::AbsolutePosition>();
    flatbuffers::Offset<frc971::AbsolutePosition> intake_offset =
        intake_pot_encoder_.GetSensorValues(&intake_builder);

    frc971::PotAndAbsolutePosition::Builder stilts_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> stilts_offset =
        stilts_pot_encoder_.GetSensorValues(&stilts_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_elevator(elevator_offset);
    position_builder.add_wrist(wrist_offset);
    position_builder.add_intake_joint(intake_offset);
    position_builder.add_stilts(stilts_offset);
    position_builder.add_suction_pressure(simulated_pressure_);

    builder.Send(position_builder.Finish());
  }

  double elevator_position() const { return elevator_plant_->X(0, 0); }
  double elevator_velocity() const { return elevator_plant_->X(1, 0); }

  double wrist_position() const { return wrist_plant_->X(0, 0); }
  double wrist_velocity() const { return wrist_plant_->X(1, 0); }

  double intake_position() const { return intake_plant_->X(0, 0); }
  double intake_velocity() const { return intake_plant_->X(1, 0); }

  double stilts_position() const { return stilts_plant_->X(0, 0); }
  double stilts_velocity() const { return stilts_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.

  void set_simulated_pressure(double pressure) {
    simulated_pressure_ = pressure;
  }

  // Simulates the superstructure for a single timestep.
  void Simulate() {
    const double last_elevator_velocity = elevator_velocity();
    const double last_wrist_velocity = wrist_velocity();
    const double last_intake_velocity = intake_velocity();
    const double last_stilts_velocity = stilts_velocity();

    EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
    EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

    if (check_collisions_) {
      CheckCollisions(superstructure_status_fetcher_.get());
    }

    const double voltage_check_elevator =
        (static_cast<PotAndAbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->elevator()->state()) ==
         PotAndAbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().elevator.subsystem_params.operating_voltage
            : constants::GetValues().elevator.subsystem_params.zeroing_voltage;

    const double voltage_check_wrist =
        (static_cast<PotAndAbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->wrist()->state()) ==
         PotAndAbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().wrist.subsystem_params.operating_voltage
            : constants::GetValues().wrist.subsystem_params.zeroing_voltage;

    const double voltage_check_intake =
        (static_cast<AbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->intake()->state()) ==
         AbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().intake.operating_voltage
            : constants::GetValues().intake.zeroing_voltage;

    const double voltage_check_stilts =
        (static_cast<PotAndAbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->stilts()->state()) ==
         PotAndAbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().stilts.subsystem_params.operating_voltage
            : constants::GetValues().stilts.subsystem_params.zeroing_voltage;

    EXPECT_NEAR(superstructure_output_fetcher_->elevator_voltage(), 0.0,
                voltage_check_elevator);

    EXPECT_NEAR(superstructure_output_fetcher_->wrist_voltage(), 0.0,
                voltage_check_wrist);

    EXPECT_NEAR(superstructure_output_fetcher_->intake_joint_voltage(), 0.0,
                voltage_check_intake);

    EXPECT_NEAR(superstructure_output_fetcher_->stilts_voltage(), 0.0,
                voltage_check_stilts);

    ::Eigen::Matrix<double, 1, 1> elevator_U;
    elevator_U << superstructure_output_fetcher_->elevator_voltage() +
                      elevator_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> wrist_U;
    wrist_U << superstructure_output_fetcher_->wrist_voltage() +
                   wrist_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> intake_U;
    intake_U << superstructure_output_fetcher_->intake_joint_voltage() +
                    intake_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> stilts_U;
    stilts_U << superstructure_output_fetcher_->stilts_voltage() +
                    stilts_plant_->voltage_offset();

    elevator_plant_->Update(elevator_U);
    wrist_plant_->Update(wrist_U);
    intake_plant_->Update(intake_U);
    stilts_plant_->Update(stilts_U);

    const double position_elevator = elevator_plant_->Y(0, 0);
    const double position_wrist = wrist_plant_->Y(0, 0);
    const double position_intake = intake_plant_->Y(0, 0);
    const double position_stilts = stilts_plant_->Y(0, 0);

    elevator_pot_encoder_.MoveTo(position_elevator);
    wrist_pot_encoder_.MoveTo(position_wrist);
    intake_pot_encoder_.MoveTo(position_intake);
    stilts_pot_encoder_.MoveTo(position_stilts);

    EXPECT_GE(position_elevator,
              constants::Values::kElevatorRange().lower_hard);
    EXPECT_LE(position_elevator,
              constants::Values::kElevatorRange().upper_hard);

    EXPECT_GE(position_wrist, constants::Values::kWristRange().lower_hard);
    EXPECT_LE(position_wrist, constants::Values::kWristRange().upper_hard);

    EXPECT_GE(position_intake, constants::Values::kIntakeRange().lower_hard);
    EXPECT_LE(position_intake, constants::Values::kIntakeRange().upper_hard);

    EXPECT_GE(position_stilts, constants::Values::kStiltsRange().lower_hard);
    EXPECT_LE(position_stilts, constants::Values::kStiltsRange().upper_hard);

    // Check that no constraints have been violated.
    if (check_collisions_) {
      CheckCollisions(superstructure_status_fetcher_.get());
    }

    const double loop_time = ::aos::time::DurationInSeconds(dt_);

    const double elevator_acceleration =
        (elevator_velocity() - last_elevator_velocity) / loop_time;
    const double wrist_acceleration =
        (wrist_velocity() - last_wrist_velocity) / loop_time;
    const double intake_acceleration =
        (intake_velocity() - last_intake_velocity) / loop_time;
    const double stilts_acceleration =
        (stilts_velocity() - last_stilts_velocity) / loop_time;

    EXPECT_GE(peak_elevator_acceleration_, elevator_acceleration);
    EXPECT_LE(-peak_elevator_acceleration_, elevator_acceleration);
    EXPECT_GE(peak_elevator_velocity_, elevator_velocity());
    EXPECT_LE(-peak_elevator_velocity_, elevator_velocity());

    EXPECT_GE(peak_wrist_acceleration_, wrist_acceleration);
    EXPECT_LE(-peak_wrist_acceleration_, wrist_acceleration);
    EXPECT_GE(peak_wrist_velocity_, wrist_velocity());
    EXPECT_LE(-peak_wrist_velocity_, wrist_velocity());

    EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
    EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);
    EXPECT_GE(peak_intake_velocity_, intake_velocity());
    EXPECT_LE(-peak_intake_velocity_, intake_velocity());

    EXPECT_GE(peak_stilts_acceleration_, stilts_acceleration);
    EXPECT_LE(-peak_stilts_acceleration_, stilts_acceleration);
    EXPECT_GE(peak_stilts_velocity_, stilts_velocity());
    EXPECT_LE(-peak_stilts_velocity_, stilts_velocity());
  }

  void set_peak_elevator_acceleration(double value) {
    peak_elevator_acceleration_ = value;
  }
  void set_peak_elevator_velocity(double value) {
    peak_elevator_velocity_ = value;
  }

  void set_peak_wrist_acceleration(double value) {
    peak_wrist_acceleration_ = value;
  }
  void set_peak_wrist_velocity(double value) { peak_wrist_velocity_ = value; }

  void set_peak_intake_acceleration(double value) {
    peak_intake_acceleration_ = value;
  }
  void set_peak_intake_velocity(double value) { peak_intake_velocity_ = value; }

  void set_peak_stilts_acceleration(double value) {
    peak_stilts_acceleration_ = value;
  }
  void set_peak_stilts_velocity(double value) { peak_stilts_velocity_ = value; }

  void set_check_collisions(bool check_collisions) {
    check_collisions_ = check_collisions;
  }

 private:
  void CheckCollisions(const Status *status) {
    ASSERT_FALSE(collision_avoidance_.IsCollided(
        wrist_position(), elevator_position(), intake_position(),
        status->has_piece()));
  }

  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool first_ = true;

  ::std::unique_ptr<CappedTestPlant> elevator_plant_;
  PositionSensorSimulator elevator_pot_encoder_;

  ::std::unique_ptr<CappedTestPlant> wrist_plant_;
  PositionSensorSimulator wrist_pot_encoder_;

  ::std::unique_ptr<CappedTestPlant> intake_plant_;
  PositionSensorSimulator intake_pot_encoder_;

  ::std::unique_ptr<CappedTestPlant> stilts_plant_;
  PositionSensorSimulator stilts_pot_encoder_;

  double simulated_pressure_ = 1.0;

  bool check_collisions_ = true;

  // The acceleration limits to check for while moving.
  double peak_elevator_acceleration_ = 1e10;
  double peak_wrist_acceleration_ = 1e10;
  double peak_intake_acceleration_ = 1e10;
  double peak_stilts_acceleration_ = 1e10;

  // The velocity limits to check for while moving.
  double peak_elevator_velocity_ = 1e10;
  double peak_wrist_velocity_ = 1e10;
  double peak_intake_velocity_ = 1e10;
  double peak_stilts_velocity_ = 1e10;

  // Creat a collision avoidance object
  CollisionAvoidance collision_avoidance_;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : ::aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2019/config.json"),
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

    EXPECT_NEAR(superstructure_goal_fetcher_->elevator()->unsafe_goal(),
                superstructure_status_fetcher_->elevator()->position(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->wrist()->unsafe_goal(),
                superstructure_plant_.wrist_position(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->intake()->unsafe_goal(),
                superstructure_status_fetcher_->intake()->position(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->stilts()->unsafe_goal(),
                superstructure_plant_.stilts_position(), 0.001);
  }

  void WaitUntilZeroed() {
    int i = 0;
    do {
      i++;
      RunFor(dt());
      superstructure_status_fetcher_.Fetch();
      // 2 Seconds
      ASSERT_LE(i, 2 * 1.0 / .00505);

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

// Tests that the superstructure does nothing when the goal is zero.
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);
  superstructure_plant_.InitializeElevatorPosition(1.4);
  superstructure_plant_.InitializeWristPosition(1.0);
  superstructure_plant_.InitializeIntakePosition(1.1);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.4);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.1);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);

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

  superstructure_plant_.InitializeElevatorPosition(1.4);
  superstructure_plant_.InitializeWristPosition(1.0);
  superstructure_plant_.InitializeIntakePosition(1.1);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.4,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.0,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.1,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
//
// We are going to disable collision detection to make this easier to implement.
TEST_F(SuperstructureTest, SaturationTest) {
  SetEnabled(true);
  // Zero it before we move.
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();


    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kElevatorRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kWristRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kStiltsRange().upper);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(8));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kElevatorRange().upper,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kWristRange().upper,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kStiltsRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_peak_elevator_velocity(23.0);
  superstructure_plant_.set_peak_elevator_acceleration(0.2);
  superstructure_plant_.set_peak_wrist_velocity(23.0);
  superstructure_plant_.set_peak_wrist_acceleration(0.2);
  superstructure_plant_.set_peak_intake_velocity(23.0);
  superstructure_plant_.set_peak_intake_acceleration(0.2);
  superstructure_plant_.set_peak_stilts_velocity(23.0);
  superstructure_plant_.set_peak_stilts_acceleration(0.2);

  RunFor(chrono::seconds(8));
  VerifyNearGoal();
}

// Tests if the robot zeroes properly... maybe redundant?
TEST_F(SuperstructureTest, ZeroTest) {
  superstructure_plant_.InitializeElevatorPosition(1.4);
  superstructure_plant_.InitializeWristPosition(1.0);
  superstructure_plant_.InitializeIntakePosition(1.1);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.4,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.0,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.1,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.5));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  WaitUntilZeroed();
  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));
  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.elevator().state());

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.wrist().state());

  EXPECT_EQ(AbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.intake().state());

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.stilts().state());
}

// Move wrist front to back and see if we collide
TEST_F(SuperstructureTest, CollisionTest) {
  SetEnabled(true);
  // Set a reasonable goal.
  superstructure_plant_.InitializeElevatorPosition(
      constants::Values::kElevatorRange().lower);
  // 60 degrees forwards
  superstructure_plant_.InitializeWristPosition(M_PI / 3.0);
  superstructure_plant_.InitializeIntakePosition(
      CollisionAvoidance::kIntakeOutAngle + CollisionAvoidance::kEpsIntake);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kElevatorRange().lower);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), -M_PI / 3.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), CollisionAvoidance::kIntakeInAngle -
                                CollisionAvoidance::kEpsIntake);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(20));

  VerifyNearGoal();
}

// Tests that the rollers spin when allowed
TEST_F(SuperstructureTest, IntakeRollerTest) {
  SetEnabled(true);
  WaitUntilZeroed();

  // Get the elevator and wrist out of the way and set the Intake to where
  // we should be able to spin and verify that they do
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kElevatorRange().upper);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);
    goal_builder.add_roller_voltage(6.0);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));
  superstructure_goal_fetcher_.Fetch();
  superstructure_output_fetcher_.Fetch();
  EXPECT_EQ(superstructure_output_fetcher_->intake_roller_voltage(),
            superstructure_goal_fetcher_->roller_voltage());
  VerifyNearGoal();

  // Move the intake where we oughtn't to spin the rollers and verify they don't
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kElevatorRange().upper);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);
    goal_builder.add_roller_voltage(6.0);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));
  superstructure_goal_fetcher_.Fetch();
  superstructure_output_fetcher_.Fetch();
  EXPECT_EQ(superstructure_output_fetcher_->intake_roller_voltage(), 0.0);
  VerifyNearGoal();
}

// Tests the Vacuum detects a gamepiece
TEST_F(SuperstructureTest, VacuumDetectsPiece) {
  SetEnabled(true);
  WaitUntilZeroed();
  // Turn on suction
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.4);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.1);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    flatbuffers::Offset<SuctionGoal> suction_offset =
        CreateSuctionGoal(*builder.fbb(), true);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);
    goal_builder.add_suction(suction_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(Vacuum::kTimeAtHigherVoltage - chrono::milliseconds(10));

  // Verify that at 0 pressure after short time voltage is still 12
  superstructure_plant_.set_simulated_pressure(0.0);
  RunFor(chrono::seconds(2));
  superstructure_status_fetcher_.Fetch();
  EXPECT_TRUE(superstructure_status_fetcher_->has_piece());
}

// Tests the Vacuum backs off after acquiring a gamepiece
TEST_F(SuperstructureTest, VacuumBacksOff) {
  SetEnabled(true);
  WaitUntilZeroed();
  // Turn on suction
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.4);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.1);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    flatbuffers::Offset<SuctionGoal> suction_offset =
        CreateSuctionGoal(*builder.fbb(), true);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);
    goal_builder.add_suction(suction_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Verify that at 0 pressure after short time voltage is still high
  superstructure_plant_.set_simulated_pressure(0.0);
  RunFor(Vacuum::kTimeAtHigherVoltage - chrono::milliseconds(10));
  superstructure_output_fetcher_.Fetch();
  EXPECT_EQ(superstructure_output_fetcher_->pump_voltage(),
            Vacuum::kPumpVoltage);

  // Verify that after waiting with a piece the pump voltage goes to the
  // has piece voltage
  RunFor(chrono::seconds(2));
  superstructure_output_fetcher_.Fetch();
  EXPECT_EQ(superstructure_output_fetcher_->pump_voltage(),
            Vacuum::kPumpHasPieceVoltage);
}

// Tests the Vacuum stops immediately after getting a no suck goal
TEST_F(SuperstructureTest, VacuumStopsQuickly) {
  SetEnabled(true);
  WaitUntilZeroed();
  // Turn on suction
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.4);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.1);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    flatbuffers::Offset<SuctionGoal> suction_offset =
        CreateSuctionGoal(*builder.fbb(), true);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);
    goal_builder.add_suction(suction_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Get a Gamepiece
  superstructure_plant_.set_simulated_pressure(0.0);
  RunFor(chrono::seconds(2));
  superstructure_status_fetcher_.Fetch();
  EXPECT_TRUE(superstructure_status_fetcher_->has_piece());

  // Turn off suction
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.4);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 1.1);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), 0.1);

    flatbuffers::Offset<SuctionGoal> suction_offset =
        CreateSuctionGoal(*builder.fbb(), false);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_elevator(elevator_offset);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_stilts(stilts_offset);
    goal_builder.add_suction(suction_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_simulated_pressure(1.0);
  // Run for a short while and check that voltage dropped to 0
  RunFor(chrono::milliseconds(10));
  superstructure_output_fetcher_.Fetch();
  EXPECT_EQ(superstructure_output_fetcher_->pump_voltage(), 0.0);
}

// Tests that running disabled, ya know, works
TEST_F(SuperstructureTest, DiasableTest) {
  superstructure_plant_.set_check_collisions(false);
  RunFor(chrono::seconds(2));
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
