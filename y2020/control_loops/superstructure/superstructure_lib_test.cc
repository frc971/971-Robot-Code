#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2020/constants.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"
#include "y2020/control_loops/superstructure/hood/hood_plant.h"
#include "y2020/control_loops/superstructure/intake/intake_plant.h"
#include "y2020/control_loops/superstructure/superstructure.h"

DEFINE_string(output_file, "",
              "If set, logs all channels to the provided logfile.");
DEFINE_string(replay_logfile, "external/superstructure_replay/",
              "Name of the logfile to read from and replay.");
DEFINE_string(config, "y2020/config.json",
              "Name of the config file to replay using.");

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
typedef ::frc971::control_loops::drivetrain::Status DrivetrainStatus;
typedef Superstructure::AbsoluteEncoderSubsystem AbsoluteEncoderSubsystem;
typedef Superstructure::AbsoluteAndAbsoluteEncoderSubsystem
    AbsoluteAndAbsoluteEncoderSubsystem;
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;

class FlywheelPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit FlywheelPlant(StateFeedbackPlant<2, 1, 1> &&other, double bemf,
                         double resistance)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)),
        bemf_(bemf),
        resistance_(resistance) {}

  void CheckU(const Eigen::Matrix<double, 1, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + voltage_offset_);
  }

  double motor_current(const Eigen::Matrix<double, 1, 1> U) const {
    return (U(0) - X(1) / bemf_) / resistance_;
  }

  double battery_current(const Eigen::Matrix<double, 1, 1> U) const {
    return motor_current(U) * U(0) / 12.0;
  }

  double voltage_offset() const { return voltage_offset_; }
  void set_voltage_offset(double voltage_offset) {
    voltage_offset_ = voltage_offset;
  }

 private:
  double voltage_offset_ = 0.0;

  double bemf_;
  double resistance_;
};

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
        hood_encoder_(
            constants::GetValues()
                .hood.zeroing_constants.one_revolution_distance,
            constants::GetValues()
                .hood.zeroing_constants.single_turn_one_revolution_distance),
        intake_plant_(new CappedTestPlant(intake::MakeIntakePlant())),
        intake_encoder_(constants::GetValues()
                            .intake.zeroing_constants.one_revolution_distance),
        turret_plant_(new CappedTestPlant(turret::MakeTurretPlant())),
        turret_encoder_(constants::GetValues()
                            .turret.subsystem_params.zeroing_constants
                            .one_revolution_distance),
        accelerator_left_plant_(
            new FlywheelPlant(accelerator::MakeAcceleratorPlant(),
                              accelerator::kBemf, accelerator::kResistance)),
        accelerator_right_plant_(
            new FlywheelPlant(accelerator::MakeAcceleratorPlant(),
                              accelerator::kBemf, accelerator::kResistance)),
        finisher_plant_(new FlywheelPlant(finisher::MakeFinisherPlant(),
                                          finisher::kBemf,
                                          finisher::kResistance)) {
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
            .hood.zeroing_constants.measured_absolute_position,
        constants::GetValues()
            .hood.zeroing_constants.single_turn_measured_absolute_position);
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

  flatbuffers::Offset<ShooterPosition> shooter_pos_offset(
      ShooterPositionBuilder *builder) {
    builder->add_theta_finisher(finisher_plant_->Y(0, 0));
    builder->add_theta_accelerator_left(accelerator_left_plant_->Y(0, 0));
    builder->add_theta_accelerator_right(accelerator_right_plant_->Y(0, 0));
    return builder->Finish();
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::AbsoluteAndAbsolutePosition::Builder hood_builder =
        builder.MakeBuilder<frc971::AbsoluteAndAbsolutePosition>();
    flatbuffers::Offset<frc971::AbsoluteAndAbsolutePosition> hood_offset =
        hood_encoder_.GetSensorValues(&hood_builder);

    frc971::AbsolutePosition::Builder intake_builder =
        builder.MakeBuilder<frc971::AbsolutePosition>();
    flatbuffers::Offset<frc971::AbsolutePosition> intake_offset =
        intake_encoder_.GetSensorValues(&intake_builder);

    frc971::PotAndAbsolutePosition::Builder turret_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> turret_offset =
        turret_encoder_.GetSensorValues(&turret_builder);

    ShooterPosition::Builder shooter_builder =
        builder.MakeBuilder<ShooterPosition>();
    flatbuffers::Offset<ShooterPosition> shooter_offset =
        shooter_pos_offset(&shooter_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_hood(hood_offset);
    position_builder.add_intake_joint(intake_offset);
    position_builder.add_turret(turret_offset);
    position_builder.add_shooter(shooter_offset);
    position_builder.add_intake_beambreak_triggered(
        intake_beambreak_triggered_);

    builder.Send(position_builder.Finish());
  }

  double hood_position() const { return hood_plant_->X(0, 0); }
  double hood_velocity() const { return hood_plant_->X(1, 0); }

  double intake_position() const { return intake_plant_->X(0, 0); }
  double intake_velocity() const { return intake_plant_->X(1, 0); }

  double turret_position() const { return turret_plant_->X(0, 0); }
  double turret_velocity() const { return turret_plant_->X(1, 0); }

  double accelerator_left_velocity() const {
    return accelerator_left_plant_->X(1, 0);
  }

  double accelerator_right_velocity() const {
    return accelerator_right_plant_->X(1, 0);
  }

  double finisher_velocity() const { return finisher_plant_->X(1, 0); }

  // Simulates the superstructure for a single timestep.
  void Simulate() {
    const double last_hood_velocity = hood_velocity();
    const double last_intake_velocity = intake_velocity();
    const double last_turret_velocity = turret_velocity();

    EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
    EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

    const double voltage_check_hood =
        (static_cast<AbsoluteAndAbsoluteEncoderSubsystem::State>(
             superstructure_status_fetcher_->hood()->state()) ==
         AbsoluteAndAbsoluteEncoderSubsystem::State::RUNNING)
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

    // Invert the friction model.
    ::Eigen::Matrix<double, 1, 1> hood_U;
    hood_U << superstructure_output_fetcher_->hood_voltage() +
                  hood_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> intake_U;
    intake_U << superstructure_output_fetcher_->intake_joint_voltage() +
                    intake_plant_->voltage_offset();

    // Invert the friction model.
    const double turret_velocity_sign =
        turret_plant_->X(1) * Superstructure::kTurretFrictionGain;
    ::Eigen::Matrix<double, 1, 1> turret_U;
    turret_U << superstructure_output_fetcher_->turret_voltage() +
                    turret_plant_->voltage_offset() -
                    std::clamp(turret_velocity_sign,
                               -Superstructure::kTurretFrictionVoltageLimit,
                               Superstructure::kTurretFrictionVoltageLimit);

    ::Eigen::Matrix<double, 1, 1> accelerator_left_U;
    accelerator_left_U
        << superstructure_output_fetcher_->accelerator_left_voltage() +
               accelerator_left_plant_->voltage_offset();

    // Confirm that we aren't drawing too much current.
    CHECK_NEAR(accelerator_left_plant_->battery_current(accelerator_left_U),
               0.0, 75.0);

    ::Eigen::Matrix<double, 1, 1> accelerator_right_U;
    accelerator_right_U
        << superstructure_output_fetcher_->accelerator_right_voltage() +
               accelerator_right_plant_->voltage_offset();

    // Confirm that we aren't drawing too much current.
    CHECK_NEAR(accelerator_right_plant_->battery_current(accelerator_right_U),
               0.0, 75.0);

    ::Eigen::Matrix<double, 1, 1> finisher_U;
    finisher_U << superstructure_output_fetcher_->finisher_voltage() +
                      finisher_plant_->voltage_offset();

    // Confirm that we aren't drawing too much current.  2 motors -> twice the
    // lumped current since our model can't tell them apart.
    CHECK_NEAR(finisher_plant_->battery_current(finisher_U), 0.0, 200.0);

    hood_plant_->Update(hood_U);
    intake_plant_->Update(intake_U);
    turret_plant_->Update(turret_U);
    accelerator_left_plant_->Update(accelerator_left_U);
    accelerator_right_plant_->Update(accelerator_right_U);
    finisher_plant_->Update(finisher_U);

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
  void set_finisher_voltage_offset(double value) {
    finisher_plant_->set_voltage_offset(value);
  }

  void set_intake_beambreak_triggered(bool triggered) {
    intake_beambreak_triggered_ = triggered;
  }

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

  ::std::unique_ptr<FlywheelPlant> accelerator_left_plant_;
  ::std::unique_ptr<FlywheelPlant> accelerator_right_plant_;
  ::std::unique_ptr<FlywheelPlant> finisher_plant_;

  // The acceleration limits to check for while moving.
  double peak_hood_acceleration_ = 1e10;
  double peak_intake_acceleration_ = 1e10;
  double peak_turret_acceleration_ = 1e10;

  // The velocity limits to check for while moving.
  double peak_hood_velocity_ = 1e10;
  double peak_intake_velocity_ = 1e10;
  double peak_turret_velocity_ = 1e10;

  float climber_voltage_ = 0.0f;

  bool intake_beambreak_triggered_ = false;
};

class SuperstructureTestEnvironment : public ::testing::Environment {
 public:
  void SetUp() override { constants::InitValues(); }
};

namespace {
const auto kTestEnv =
    ::testing::AddGlobalTestEnvironment(new SuperstructureTestEnvironment());
}

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2020/config.json"),
            chrono::microseconds(5050)),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        test_event_loop_(MakeEventLoop("test", roborio_)),
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
        drivetrain_status_sender_(
            test_event_loop_->MakeSender<DrivetrainStatus>("/drivetrain")),
        joystick_state_sender_(
            test_event_loop_->MakeSender<aos::JoystickState>("/aos")),
        superstructure_event_loop_(MakeEventLoop("superstructure", roborio_)),
        superstructure_(superstructure_event_loop_.get()),
        superstructure_plant_event_loop_(MakeEventLoop("plant", roborio_)),
        superstructure_plant_(superstructure_plant_event_loop_.get(), dt()) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);

    if (!FLAGS_output_file.empty()) {
      unlink(FLAGS_output_file.c_str());
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_file);
    }
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

    if (superstructure_goal_fetcher_->has_shooter()) {
      EXPECT_NEAR(
          superstructure_goal_fetcher_->shooter()->velocity_accelerator(),
          superstructure_status_fetcher_->shooter()
              ->accelerator_left()
              ->angular_velocity(),
          0.001);

      EXPECT_NEAR(
          superstructure_goal_fetcher_->shooter()->velocity_accelerator(),
          superstructure_status_fetcher_->shooter()
              ->accelerator_right()
              ->angular_velocity(),
          0.001);

      EXPECT_NEAR(superstructure_goal_fetcher_->shooter()->velocity_finisher(),
                  superstructure_status_fetcher_->shooter()
                      ->finisher()
                      ->angular_velocity(),
                  0.001);

      EXPECT_NEAR(
          superstructure_goal_fetcher_->shooter()->velocity_accelerator(),
          superstructure_status_fetcher_->shooter()
              ->accelerator_left()
              ->avg_angular_velocity(),
          0.001);

      EXPECT_NEAR(
          superstructure_goal_fetcher_->shooter()->velocity_accelerator(),
          superstructure_status_fetcher_->shooter()
              ->accelerator_right()
              ->avg_angular_velocity(),
          0.001);

      EXPECT_NEAR(superstructure_goal_fetcher_->shooter()->velocity_finisher(),
                  superstructure_status_fetcher_->shooter()
                      ->finisher()
                      ->avg_angular_velocity(),
                  0.001);
    }
    EXPECT_FALSE(superstructure_status_fetcher_->has_subsystems_not_ready());
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

      // Since there is a delay when sending running, make sure we have a
      // status before checking it.
    } while (superstructure_status_fetcher_.get() == nullptr ||
             !superstructure_status_fetcher_.get()->zeroed());
  }

  // Method for testing the intake_roller_voltage
  void TestIntakeRollerVoltage(const float roller_voltage,
                               const float roller_speed_compensation,
                               const bool shooting,
                               const double expected_voltage) {
    SetEnabled(true);
    {
      auto builder = superstructure_goal_sender_.MakeBuilder();
      Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
      goal_builder.add_roller_voltage(roller_voltage);
      goal_builder.add_roller_speed_compensation(roller_speed_compensation);
      goal_builder.add_shooting(shooting);
      ASSERT_TRUE(builder.Send(goal_builder.Finish()));
    }
    RunFor(chrono::seconds(1));
    superstructure_output_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_output_fetcher_.get() != nullptr);
    EXPECT_EQ(superstructure_output_fetcher_->intake_roller_voltage(),
              expected_voltage);
  }

  void StartSendingFinisherGoals() {
    test_event_loop_->AddPhasedLoop(
        [this](int) {
          auto builder = superstructure_goal_sender_.MakeBuilder();
          auto shooter_goal_builder = builder.MakeBuilder<ShooterGoal>();
          shooter_goal_builder.add_velocity_finisher(finisher_goal_);
          const auto shooter_goal_offset = shooter_goal_builder.Finish();

          Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
          goal_builder.add_shooter(shooter_goal_offset);
          ASSERT_TRUE(builder.Send(goal_builder.Finish()));
        },
        dt());
  }

  // Sets the finisher velocity goal (radians/s)
  void SetFinisherGoalAfter(const double velocity_finisher,
                            const monotonic_clock::duration time_offset) {
    test_event_loop_
        ->AddTimer(
            [this, velocity_finisher] { finisher_goal_ = velocity_finisher; })
        ->Setup(test_event_loop_->monotonic_now() + time_offset);
  }

  // Simulates the friction of a ball in the flywheel
  void ApplyFrictionToFinisherAfter(
      const double voltage_offset, const bool ball_in_finisher,
      const monotonic_clock::duration time_offset) {
    test_event_loop_
        ->AddTimer([this, voltage_offset, ball_in_finisher] {
          superstructure_plant_.set_finisher_voltage_offset(voltage_offset);
          ball_in_finisher_ = ball_in_finisher;
        })
        ->Setup(test_event_loop_->monotonic_now() + time_offset);
    test_event_loop_
        ->AddTimer(
            [this] { superstructure_plant_.set_finisher_voltage_offset(0); })
        ->Setup(test_event_loop_->monotonic_now() + time_offset +
                chrono::seconds(1));
  }

  const aos::Node *const roborio_;

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;
  ::aos::Sender<DrivetrainStatus> drivetrain_status_sender_;
  ::aos::Sender<aos::JoystickState> joystick_state_sender_;

  // Create a control loop and simulation.
  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop_;
  Superstructure superstructure_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;

  double finisher_goal_ = 0;
  bool ball_in_finisher_ = false;
};

// Tests that the superstructure does nothing when the goal is to remain
// still.
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

    flatbuffers::Offset<ShooterGoal> shooter_offset =
        CreateShooterGoal(*builder.fbb(), 0.0, 0.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_shooter(shooter_offset);

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

  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange().middle());
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().middle());

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().middle() + 1.0);

    flatbuffers::Offset<ShooterGoal> shooter_offset =
        CreateShooterGoal(*builder.fbb(), 300.0, 300.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating)
// behaviour.
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

    flatbuffers::Offset<ShooterGoal> shooter_offset =
        CreateShooterGoal(*builder.fbb(), 0.0, 0.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_shooter(shooter_offset);

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

    flatbuffers::Offset<ShooterGoal> shooter_offset =
        CreateShooterGoal(*builder.fbb(), 0.0, 0.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_peak_hood_velocity(23.0);
  // 30 hz sin wave on the hood causes acceleration to be ignored.
  superstructure_plant_.set_peak_hood_acceleration(5.5);

  superstructure_plant_.set_peak_intake_velocity(23.0);
  superstructure_plant_.set_peak_intake_acceleration(0.2);

  superstructure_plant_.set_peak_turret_velocity(23.0);
  superstructure_plant_.set_peak_turret_acceleration(6.0);

  // Intake needs over 9 seconds to reach the goal
  RunFor(chrono::seconds(10));
  VerifyNearGoal();
}

// Tests the shooter can spin up correctly.
TEST_F(SuperstructureTest, SpinUp) {
  SetEnabled(true);
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange().upper);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange().upper);

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();

    // Start up the accelerator and make sure both run.
    shooter_builder.add_velocity_accelerator(200.0);
    shooter_builder.add_velocity_finisher(200.0);

    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_shooting(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // In the beginning, the finisher and accelerator should not be ready
  test_event_loop_
      ->AddTimer([&]() {
        ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
        ASSERT_TRUE(superstructure_status_fetcher_->has_subsystems_not_ready());
        const auto subsystems_not_ready =
            superstructure_status_fetcher_->subsystems_not_ready();
        ASSERT_EQ(subsystems_not_ready->size(), 2);
        EXPECT_TRUE((subsystems_not_ready->Get(0) == Subsystem::FINISHER) !=
                    (subsystems_not_ready->Get(1) == Subsystem::FINISHER));
        EXPECT_TRUE((subsystems_not_ready->Get(0) == Subsystem::ACCELERATOR) !=
                    (subsystems_not_ready->Get(1) == Subsystem::ACCELERATOR));
      })
      ->Setup(test_event_loop_->monotonic_now() + chrono::milliseconds(1));

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<ShooterGoal> shooter_offset =
        CreateShooterGoal(*builder.fbb(), 0.0, 0.0);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_hood(hood_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(25));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));
  EXPECT_EQ(AbsoluteAndAbsoluteEncoderSubsystem::State::RUNNING,
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

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_voltage(-10.0);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it time to stabilize.
  RunFor(chrono::seconds(1));

  // Can go backwards.
  EXPECT_EQ(superstructure_plant_.climber_voltage(), -10.0);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_voltage(10.0);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(1));

  // And forwards too.
  EXPECT_EQ(superstructure_plant_.climber_voltage(), 10.0);

  VerifyNearGoal();
}

// Tests that preserializing balls works.
TEST_F(SuperstructureTest, Preserializing) {
  SetEnabled(true);
  // Set a reasonable goal.

  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_preloading(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_intake_beambreak_triggered(false);

  // Give it time to stabilize.
  RunFor(chrono::seconds(1));

  // Preloads balls.
  superstructure_output_fetcher_.Fetch();
  ASSERT_TRUE(superstructure_output_fetcher_.get() != nullptr);
  EXPECT_EQ(superstructure_output_fetcher_->feeder_voltage(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->washing_machine_spinner_voltage(),
            5.0);

  VerifyNearGoal();

  superstructure_plant_.set_intake_beambreak_triggered(true);

  // Give it time to stabilize.
  RunFor(chrono::seconds(1));

  // Stops preloading balls once one ball is in place
  superstructure_output_fetcher_.Fetch();
  ASSERT_TRUE(superstructure_output_fetcher_.get() != nullptr);
  EXPECT_EQ(superstructure_output_fetcher_->feeder_voltage(), 0.0);
  EXPECT_EQ(superstructure_output_fetcher_->washing_machine_spinner_voltage(),
            0.0);

  VerifyNearGoal();
}

// Makes sure that a negative number is not added to the to the
// roller_voltage
TEST_F(SuperstructureTest, NegativeRollerSpeedCompensation) {
  constexpr float kRollerVoltage = 12.0f;
  TestIntakeRollerVoltage(kRollerVoltage, -10.0f, false, kRollerVoltage);
}

// Makes sure that intake_roller_voltage is correctly being calculated
// based on the velocity when the roller_speed_compensation is positive
TEST_F(SuperstructureTest, PositiveRollerSpeedCompensation) {
  constexpr float kRollerVoltage = 12.0f;
  constexpr float kRollerSpeedCompensation = 10.0f;
  TestIntakeRollerVoltage(kRollerVoltage, kRollerSpeedCompensation, false,
                          kRollerVoltage + (superstructure_.robot_speed() *
                                            kRollerSpeedCompensation));
}

// Makes sure that the intake_roller_voltage is 3.0
// when the robot should be shooting balls (after the hood, turret, and
// shooter at at the goal)
TEST_F(SuperstructureTest, WhenShooting) {
  TestIntakeRollerVoltage(0.0f, 0.0f, true, 3.0);
}

// Tests that we detect that a ball was shot whenever the  average angular
// velocity is lower than a certain threshold compared to the goal.
TEST_F(SuperstructureTest, BallsShot) {
  SetEnabled(true);
  WaitUntilZeroed();

  int balls_shot = 0;
  // When there is a ball in the flywheel, the finisher velocity should drop
  // below the goal
  bool finisher_velocity_below_goal = false;

  StartSendingFinisherGoals();

  test_event_loop_->AddPhasedLoop(
      [&](int) {
        ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
        const double finisher_velocity =
            superstructure_status_fetcher_->shooter()
                ->finisher()
                ->angular_velocity();
        const double finisher_velocity_dip = finisher_goal_ - finisher_velocity;
        if (ball_in_finisher_ &&
            finisher_velocity_dip >=
                shooter::Shooter::kVelocityToleranceFinisher) {
          finisher_velocity_below_goal = true;
        }
        if (ball_in_finisher_ && finisher_velocity_below_goal &&
            finisher_velocity_dip <
                shooter::Shooter::kVelocityToleranceFinisher) {
          ball_in_finisher_ = false;
          finisher_velocity_below_goal = false;
          balls_shot++;
        }
        // Since here we are calculating the dip from the goal instead of the
        // local maximum, the shooter could have calculated that a ball was shot
        // slightly before we did if the local maximum was slightly below the
        // goal.
        EXPECT_TRUE((superstructure_status_fetcher_->shooter()->balls_shot() ==
                     balls_shot) ||
                    ((superstructure_status_fetcher_->shooter()->balls_shot() ==
                      balls_shot + 1) &&
                     (finisher_velocity_dip -
                          shooter::Shooter::kVelocityToleranceFinisher <
                      0.2)));
      },
      dt());

  constexpr int kFastShootingSpeed = 500;
  constexpr int kSlowShootingSpeed = 300;

  // Maximum (since it is negative) flywheel voltage offsets for simulating the
  // friction of a ball at different finisher speeds.
  // Slower speeds require a higher magnitude of voltage offset.
  static constexpr double kFastSpeedVoltageOffsetWithBall = -4.1;
  static constexpr double kSlowSpeedVoltageOffsetWithBall = -4.5;

  SetFinisherGoalAfter(kFastShootingSpeed, chrono::seconds(1));
  // Simulate shooting balls by applying friction to the finisher
  ApplyFrictionToFinisherAfter(kFastSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(3));
  ApplyFrictionToFinisherAfter(kFastSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(6));

  SetFinisherGoalAfter(0, chrono::seconds(10));
  SetFinisherGoalAfter(kFastShootingSpeed, chrono::seconds(15));
  ApplyFrictionToFinisherAfter(kFastSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(18));
  ApplyFrictionToFinisherAfter(kFastSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(21));

  SetFinisherGoalAfter(kSlowShootingSpeed, chrono::seconds(25));
  ApplyFrictionToFinisherAfter(kSlowSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(28));
  ApplyFrictionToFinisherAfter(kSlowSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(31));
  // This smaller decrease in velocity shouldn't be counted as a ball
  ApplyFrictionToFinisherAfter(kSlowSpeedVoltageOffsetWithBall / 2, false,
                               chrono::seconds(34));

  SetFinisherGoalAfter(kFastShootingSpeed, chrono::seconds(38));
  ApplyFrictionToFinisherAfter(kFastSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(41));
  ApplyFrictionToFinisherAfter(kFastSpeedVoltageOffsetWithBall, true,
                               chrono::seconds(44));
  // This slow positive voltage offset that speeds up the flywheel instead of
  // slowing it down shouldn't be counted as a ball.
  // We wouldn't expect a positive voltage offset of more than ~2 volts.
  ApplyFrictionToFinisherAfter(2, false, chrono::seconds(47));

  RunFor(chrono::seconds(50));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->shooter()->balls_shot(), 8);
}

class SuperstructureReplayTest : public ::testing::Test {
 public:
  SuperstructureReplayTest()
      : config_(aos::configuration::ReadConfig(FLAGS_config)),
        reader_(
            aos::logger::SortParts(aos::logger::FindLogs(FLAGS_replay_logfile)),
            &config_.message()) {
    aos::network::OverrideTeamNumber(971);

    reader_.RemapLoggedChannel("/superstructure",
                               "y2020.control_loops.superstructure.Status");
    reader_.RemapLoggedChannel("/superstructure",
                               "y2020.control_loops.superstructure.Output");
    reader_.Register();

    roborio_ = aos::configuration::GetNode(reader_.configuration(), "roborio");

    superstructure_event_loop_ =
        reader_.event_loop_factory()->MakeEventLoop("superstructure", roborio_);
    superstructure_event_loop_->SkipTimingReport();

    test_event_loop_ = reader_.event_loop_factory()->MakeEventLoop(
        "superstructure_replay_test", roborio_);

    status_fetcher_ = test_event_loop_->MakeFetcher<Status>("/superstructure");
    goal_sender_ = test_event_loop_->MakeSender<Goal>("/superstructure");

    if (!FLAGS_output_file.empty()) {
      unlink(FLAGS_output_file.c_str());
      logger_event_loop_ =
          reader_.event_loop_factory()->MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_file);
    }
  }

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::logger::LogReader reader_;
  const aos::Node *roborio_;

  std::unique_ptr<aos::EventLoop> superstructure_event_loop_;
  std::unique_ptr<aos::EventLoop> test_event_loop_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;

  aos::Sender<y2020::control_loops::superstructure::Goal> goal_sender_;
  aos::Fetcher<y2020::control_loops::superstructure::Status> status_fetcher_;
};

// Tests that balls_shot is updated correctly with a real log.
TEST_F(SuperstructureReplayTest, BallsShotReplay) {
  Superstructure superstructure(superstructure_event_loop_.get());
  // TODO(milind): remove this after the new log is uploaded
  test_event_loop_->AddPhasedLoop(
      [&](int) {
        auto builder = goal_sender_.MakeBuilder();
        auto shooter_goal_builder = builder.MakeBuilder<ShooterGoal>();
        shooter_goal_builder.add_velocity_finisher(304);
        const auto shooter_goal_offset = shooter_goal_builder.Finish();

        auto goal_builder = builder.MakeBuilder<Goal>();
        goal_builder.add_shooter(shooter_goal_offset);
        goal_builder.add_shooter_tracking(false);
        builder.Send(goal_builder.Finish());
      },
      frc971::controls::kLoopFrequency);

  reader_.event_loop_factory()->Run();

  ASSERT_TRUE(status_fetcher_.Fetch());
  EXPECT_EQ(status_fetcher_->shooter()->balls_shot(), 3);
}

class SuperstructureAllianceTest
    : public SuperstructureTest,
      public ::testing::WithParamInterface<aos::Alliance> {
 protected:
  void SendAlliancePosition() {
    auto builder = joystick_state_sender_.MakeBuilder();

    aos::JoystickState::Builder joystick_builder =
        builder.MakeBuilder<aos::JoystickState>();

    joystick_builder.add_alliance(GetParam());

    ASSERT_TRUE(builder.Send(joystick_builder.Finish()));
  }
};

// Tests that the turret switches to auto-aiming when we set turret_tracking to
// true.
TEST_P(SuperstructureAllianceTest, TurretAutoAim) {
  SetEnabled(true);
  // Set a reasonable goal.
  const frc971::control_loops::Pose target = turret::OuterPortPose(GetParam());

  WaitUntilZeroed();

  constexpr double kShotAngle = 1.0;
  SendAlliancePosition();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_turret_tracking(true);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  {
    auto builder = drivetrain_status_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::LocalizerState::Builder
        localizer_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerState>();
    localizer_builder.add_left_velocity(0.0);
    localizer_builder.add_right_velocity(0.0);
    const auto localizer_offset = localizer_builder.Finish();

    DrivetrainStatus::Builder status_builder =
        builder.MakeBuilder<DrivetrainStatus>();

    // Set the robot up at kShotAngle off from the target, 1m away.
    status_builder.add_x(target.abs_pos().x() + std::cos(kShotAngle));
    status_builder.add_y(target.abs_pos().y() + std::sin(kShotAngle));
    status_builder.add_theta(0.0);
    status_builder.add_localizer(localizer_offset);

    ASSERT_TRUE(builder.Send(status_builder.Finish()));
  }

  // Give it time to stabilize.
  RunFor(chrono::seconds(1));

  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(kShotAngle, superstructure_status_fetcher_->turret()->position(),
              5e-4);
  EXPECT_FLOAT_EQ(kShotAngle,
                  superstructure_status_fetcher_->aimer()->turret_position());
  EXPECT_FLOAT_EQ(0,
                  superstructure_status_fetcher_->aimer()->turret_velocity());
}

// Test a manual goal
TEST_P(SuperstructureAllianceTest, ShooterInterpolationManualGoal) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    auto shooter_goal = CreateShooterGoal(*builder.fbb(), 400.0, 500.0);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().lower);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter(shooter_goal);
    goal_builder.add_hood(hood_offset);

    builder.Send(goal_builder.Finish());
  }

  RunFor(chrono::seconds(10));

  superstructure_status_fetcher_.Fetch();

  EXPECT_DOUBLE_EQ(superstructure_status_fetcher_->shooter()
                       ->accelerator_left()
                       ->angular_velocity_goal(),
                   400.0);
  EXPECT_DOUBLE_EQ(superstructure_status_fetcher_->shooter()
                       ->accelerator_right()
                       ->angular_velocity_goal(),
                   400.0);
  EXPECT_DOUBLE_EQ(superstructure_status_fetcher_->shooter()
                       ->finisher()
                       ->angular_velocity_goal(),
                   500.0);
  EXPECT_NEAR(superstructure_status_fetcher_->hood()->position(),
              constants::Values::kHoodRange().lower, 0.001);
}

// Test an out of range value with auto tracking
TEST_P(SuperstructureAllianceTest, ShooterInterpolationOutOfRange) {
  SetEnabled(true);
  const frc971::control_loops::Pose target = turret::OuterPortPose(GetParam());
  WaitUntilZeroed();
  constexpr double kShotAngle = 1.0;
  {
    auto builder = drivetrain_status_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::LocalizerState::Builder
        localizer_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerState>();
    localizer_builder.add_left_velocity(0.0);
    localizer_builder.add_right_velocity(0.0);
    const auto localizer_offset = localizer_builder.Finish();

    DrivetrainStatus::Builder status_builder =
        builder.MakeBuilder<DrivetrainStatus>();

    // Set the robot up at kShotAngle off from the target, 100m away.
    status_builder.add_x(target.abs_pos().x() + std::cos(kShotAngle) * 100);
    status_builder.add_y(target.abs_pos().y() + std::sin(kShotAngle) * 100);
    status_builder.add_theta(0.0);
    status_builder.add_localizer(localizer_offset);

    ASSERT_TRUE(builder.Send(status_builder.Finish()));
  }
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    // Add a goal, this should be ignored with auto tracking
    auto shooter_goal = CreateShooterGoal(*builder.fbb(), 400.0, 500.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().lower);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter(shooter_goal);
    goal_builder.add_hood(hood_offset);
    goal_builder.add_shooter_tracking(true);
    goal_builder.add_hood_tracking(true);

    builder.Send(goal_builder.Finish());
  }
  RunFor(chrono::seconds(10));

  superstructure_status_fetcher_.Fetch();

  EXPECT_DOUBLE_EQ(superstructure_status_fetcher_->shooter()
                       ->accelerator_left()
                       ->angular_velocity_goal(),
                   0.0);
  EXPECT_DOUBLE_EQ(superstructure_status_fetcher_->shooter()
                       ->accelerator_right()
                       ->angular_velocity_goal(),
                   0.0);
  EXPECT_DOUBLE_EQ(superstructure_status_fetcher_->shooter()
                       ->finisher()
                       ->angular_velocity_goal(),
                   0.0);
  EXPECT_NEAR(superstructure_status_fetcher_->hood()->position(),
              constants::Values::kHoodRange().upper, 0.001);
}

// Test a value in range with auto tracking
TEST_P(SuperstructureAllianceTest, ShooterInterpolationInRange) {
  SetEnabled(true);
  const frc971::control_loops::Pose target = turret::OuterPortPose(GetParam());
  WaitUntilZeroed();
  constexpr double kShotAngle = 1.0;

  SendAlliancePosition();

  // Test an in range value returns a reasonable result
  {
    auto builder = drivetrain_status_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::LocalizerState::Builder
        localizer_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerState>();
    localizer_builder.add_left_velocity(0.0);
    localizer_builder.add_right_velocity(0.0);
    const auto localizer_offset = localizer_builder.Finish();

    DrivetrainStatus::Builder status_builder =
        builder.MakeBuilder<DrivetrainStatus>();

    // Set the robot up at kShotAngle off from the target, 2.5m away.
    status_builder.add_x(target.abs_pos().x() + std::cos(kShotAngle) * 2.5);
    status_builder.add_y(target.abs_pos().y() + std::sin(kShotAngle) * 2.5);
    status_builder.add_theta(0.0);
    status_builder.add_localizer(localizer_offset);

    ASSERT_TRUE(builder.Send(status_builder.Finish()));
  }
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    // Add a goal, this should be ignored with auto tracking
    auto shooter_goal = CreateShooterGoal(*builder.fbb(), 400.0, 500.0);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kHoodRange().lower);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter(shooter_goal);
    goal_builder.add_hood(hood_offset);
    goal_builder.add_shooter_tracking(true);
    goal_builder.add_hood_tracking(true);

    builder.Send(goal_builder.Finish());
  }
  RunFor(chrono::seconds(10));

  superstructure_status_fetcher_.Fetch();

  EXPECT_GE(superstructure_status_fetcher_->shooter()
                ->accelerator_left()
                ->angular_velocity_goal(),
            250.0);
  EXPECT_GE(superstructure_status_fetcher_->shooter()
                ->accelerator_right()
                ->angular_velocity_goal(),
            250.0);
  EXPECT_DOUBLE_EQ(superstructure_status_fetcher_->shooter()
                       ->finisher()
                       ->angular_velocity_goal(),
                   350.0);
  EXPECT_GE(superstructure_status_fetcher_->hood()->position(),
            constants::Values::kHoodRange().lower);
}

INSTANTIATE_TEST_SUITE_P(ShootAnyAlliance, SuperstructureAllianceTest,
                         ::testing::Values(aos::Alliance::kRed,
                                           aos::Alliance::kBlue,
                                           aos::Alliance::kInvalid));

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
