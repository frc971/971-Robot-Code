#include <chrono>
#include <memory>

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2022/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022/control_loops/superstructure/catapult/catapult_plant.h"
#include "y2022/control_loops/superstructure/climber/climber_plant.h"
#include "y2022/control_loops/superstructure/intake/intake_plant.h"
#include "y2022/control_loops/superstructure/superstructure.h"
#include "y2022/control_loops/superstructure/turret/turret_plant.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace testing {
namespace chrono = std::chrono;
namespace {
constexpr double kNoiseScalar = 0.01;
}

using ::aos::monotonic_clock;
using ::frc971::CreateProfileParameters;
using ::frc971::control_loops::CappedTestPlant;
using ::frc971::control_loops::
    CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using ::frc971::control_loops::PositionSensorSimulator;
using ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;
typedef Superstructure::RelativeEncoderSubsystem RelativeEncoderSubsystem;
using DrivetrainStatus = ::frc971::control_loops::drivetrain::Status;

template <typename SubsystemStatus, typename SubsystemState,
          typename SubsystemConstants>
class SubsystemSimulator {
 public:
  SubsystemSimulator(CappedTestPlant *plant, PositionSensorSimulator encoder,
                     const SubsystemConstants subsystem_constants,
                     const frc971::constants::Range range,
                     double encoder_offset, const chrono::nanoseconds dt)
      : plant_(plant),
        encoder_(encoder),
        subsystem_constants_(subsystem_constants),
        range_(range),
        encoder_offset_(encoder_offset),
        dt_(dt) {}

  void InitializePosition(double start_pos) {
    plant_->mutable_X(0, 0) = start_pos;
    plant_->mutable_X(1, 0) = 0.0;

    encoder_.Initialize(start_pos, kNoiseScalar, 0.0, encoder_offset_);
  }

  // Simulates the superstructure for a single timestep.
  void Simulate(double voltage, const SubsystemStatus *status) {
    double last_velocity = plant_->X(1, 0);

    const double voltage_check =
        (static_cast<SubsystemState>(status->state()) ==
         SubsystemState::RUNNING)
            ? subsystem_constants_.subsystem_params.operating_voltage
            : subsystem_constants_.subsystem_params.zeroing_voltage;

    EXPECT_NEAR(voltage, 0.0, voltage_check);

    ::Eigen::Matrix<double, 1, 1> U;
    U << voltage + plant_->voltage_offset();
    plant_->Update(U);

    const double position = plant_->Y(0, 0);

    encoder_.MoveTo(position);

    EXPECT_GE(position, range_.lower_hard);
    EXPECT_LE(position, range_.upper_hard);

    const double loop_time = ::aos::time::DurationInSeconds(dt_);

    const double velocity = plant_->X(1, 0);
    const double acceleration = (velocity - last_velocity) / loop_time;

    EXPECT_GE(peak_acceleration_, acceleration);
    EXPECT_LE(-peak_acceleration_, acceleration);
    EXPECT_GE(peak_velocity_, velocity);
    EXPECT_LE(-peak_velocity_, velocity);
  }

  void set_peak_acceleration(double value) { peak_acceleration_ = value; }
  void set_peak_velocity(double value) { peak_velocity_ = value; }

  void set_controller_index(size_t index) { plant_->set_index(index); }

  PositionSensorSimulator *encoder() { return &encoder_; }

 private:
  std::unique_ptr<CappedTestPlant> plant_;
  PositionSensorSimulator encoder_;
  const SubsystemConstants subsystem_constants_;
  const frc971::constants::Range range_;

  double encoder_offset_ = 0.0;

  double peak_velocity_ = std::numeric_limits<double>::infinity();
  double peak_acceleration_ = std::numeric_limits<double>::infinity();

  const chrono::nanoseconds dt_;
};

using PotAndAbsoluteEncoderSimulator = SubsystemSimulator<
    frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
    PotAndAbsoluteEncoderSubsystem::State,
    constants::Values::PotAndAbsEncoderConstants>;
using RelativeEncoderSimulator = SubsystemSimulator<
    frc971::control_loops::RelativeEncoderProfiledJointStatus,
    RelativeEncoderSubsystem::State, constants::Values::PotConstants>;

// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation(::aos::EventLoop *event_loop,
                           std::shared_ptr<const constants::Values> values,
                           chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")),
        intake_front_(new CappedTestPlant(intake::MakeIntakePlant()),
                      PositionSensorSimulator(
                          values->intake_front.subsystem_params
                              .zeroing_constants.one_revolution_distance),
                      values->intake_front, constants::Values::kIntakeRange(),
                      values->intake_front.subsystem_params.zeroing_constants
                          .measured_absolute_position,
                      dt_),
        intake_back_(new CappedTestPlant(intake::MakeIntakePlant()),
                     PositionSensorSimulator(
                         values->intake_back.subsystem_params.zeroing_constants
                             .one_revolution_distance),
                     values->intake_back, constants::Values::kIntakeRange(),
                     values->intake_back.subsystem_params.zeroing_constants
                         .measured_absolute_position,
                     dt_),
        turret_(new CappedTestPlant(turret::MakeTurretPlant()),
                PositionSensorSimulator(
                    values->turret.subsystem_params.zeroing_constants
                        .one_revolution_distance),
                values->turret, constants::Values::kTurretRange(),
                values->turret.subsystem_params.zeroing_constants
                    .measured_absolute_position,
                dt_),
        catapult_(new CappedTestPlant(catapult::MakeCatapultPlant()),
                  PositionSensorSimulator(
                      values->catapult.subsystem_params.zeroing_constants
                          .one_revolution_distance),
                  values->catapult, constants::Values::kCatapultRange(),
                  values->catapult.subsystem_params.zeroing_constants
                      .measured_absolute_position,
                  dt_),
        climber_(new CappedTestPlant(climber::MakeClimberPlant()),
                 PositionSensorSimulator(
                     constants::Values::kClimberPotMetersPerRevolution()),
                 values->climber, constants::Values::kClimberRange(),
                 values->climber.potentiometer_offset, dt_) {
    intake_front_.InitializePosition(
        constants::Values::kIntakeRange().middle());
    intake_back_.InitializePosition(constants::Values::kIntakeRange().middle());
    turret_.InitializePosition(constants::Values::kTurretRange().middle());
    catapult_.InitializePosition(constants::Values::kCatapultRange().middle());
    climber_.InitializePosition(constants::Values::kClimberRange().middle());

    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
            EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

            intake_front_.Simulate(
                superstructure_output_fetcher_->intake_voltage_front(),
                superstructure_status_fetcher_->intake_front());
            intake_back_.Simulate(
                superstructure_output_fetcher_->intake_voltage_back(),
                superstructure_status_fetcher_->intake_back());
            turret_.Simulate(superstructure_output_fetcher_->turret_voltage(),
                             superstructure_status_fetcher_->turret());
            if (superstructure_status_fetcher_->mpc_active()) {
              catapult_.set_controller_index(0);
            } else {
              catapult_.set_controller_index(1);
            }
            catapult_.Simulate(
                superstructure_output_fetcher_->catapult_voltage(),
                superstructure_status_fetcher_->catapult());
            climber_.Simulate(superstructure_output_fetcher_->climber_voltage(),
                              superstructure_status_fetcher_->climber());
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::PotAndAbsolutePosition::Builder turret_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> turret_offset =
        turret_.encoder()->GetSensorValues(&turret_builder);

    frc971::PotAndAbsolutePosition::Builder catapult_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> catapult_offset =
        catapult_.encoder()->GetSensorValues(&catapult_builder);

    frc971::PotAndAbsolutePosition::Builder intake_front_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_front_offset =
        intake_front_.encoder()->GetSensorValues(&intake_front_builder);

    frc971::PotAndAbsolutePosition::Builder intake_back_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_back_offset =
        intake_back_.encoder()->GetSensorValues(&intake_back_builder);

    frc971::RelativePosition::Builder climber_builder =
        builder.MakeBuilder<frc971::RelativePosition>();
    flatbuffers::Offset<frc971::RelativePosition> climber_offset =
        climber_.encoder()->GetSensorValues(&climber_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_intake_front(intake_front_offset);
    position_builder.add_intake_back(intake_back_offset);
    position_builder.add_turret(turret_offset);
    position_builder.add_catapult(catapult_offset);
    position_builder.add_climber(climber_offset);

    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

  PotAndAbsoluteEncoderSimulator *intake_front() { return &intake_front_; }
  PotAndAbsoluteEncoderSimulator *intake_back() { return &intake_back_; }
  PotAndAbsoluteEncoderSimulator *turret() { return &turret_; }
  PotAndAbsoluteEncoderSimulator *catapult() { return &catapult_; }
  RelativeEncoderSimulator *climber() { return &climber_; }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool first_ = true;

  PotAndAbsoluteEncoderSimulator intake_front_;
  PotAndAbsoluteEncoderSimulator intake_back_;
  PotAndAbsoluteEncoderSimulator turret_;
  PotAndAbsoluteEncoderSimulator catapult_;
  RelativeEncoderSimulator climber_;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2022/aos_config.json"),
            std::chrono::microseconds(5050)),
        values_(std::make_shared<constants::Values>(constants::MakeValues(
            frc971::control_loops::testing::kTeamNumber))),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        superstructure_event_loop(MakeEventLoop("Superstructure", roborio_)),
        superstructure_(superstructure_event_loop.get(), values_),
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
        superstructure_position_sender_(
            test_event_loop_->MakeSender<Position>("/superstructure")),
        drivetrain_status_sender_(
            test_event_loop_->MakeSender<DrivetrainStatus>("/drivetrain")),
        superstructure_plant_event_loop_(MakeEventLoop("plant", roborio_)),
        superstructure_plant_(superstructure_plant_event_loop_.get(), values_,
                              dt()) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);

    SetEnabled(true);

    if (!FLAGS_output_folder.empty()) {
      unlink(FLAGS_output_folder.c_str());
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingLocalNamerOnRun(FLAGS_output_folder);
    }
  }

  void VerifyNearGoal() {
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();

    // Only check the goal if there is one.

    if (superstructure_goal_fetcher_->has_intake_front()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->intake_front()->unsafe_goal(),
                  superstructure_status_fetcher_->intake_front()->position(),
                  0.001);
    }

    if (superstructure_goal_fetcher_->has_intake_back()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->intake_back()->unsafe_goal(),
                  superstructure_status_fetcher_->intake_back()->position(),
                  0.001);
    }

    if (superstructure_goal_fetcher_->has_turret()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->turret()->unsafe_goal(),
                  superstructure_status_fetcher_->turret()->position(), 0.001);
    }

    if (superstructure_goal_fetcher_->has_catapult() &&
        superstructure_goal_fetcher_->catapult()->has_return_position()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->catapult()
                      ->return_position()
                      ->unsafe_goal(),
                  superstructure_status_fetcher_->catapult()->position(),
                  0.001);
    }

    if (superstructure_goal_fetcher_->has_climber()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->climber()->unsafe_goal(),
                  superstructure_status_fetcher_->climber()->position(), 0.001);
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

      // Since there is a delay when sending running, make sure we have a
      // status before checking it.
    } while (superstructure_status_fetcher_.get() == nullptr ||
             !superstructure_status_fetcher_.get()->zeroed());
  }

  void SendRobotVelocity(double robot_velocity) {
    // Send a robot velocity to test compensation
    auto builder = drivetrain_status_sender_.MakeBuilder();
    auto drivetrain_status_builder = builder.MakeBuilder<DrivetrainStatus>();
    drivetrain_status_builder.add_robot_speed(robot_velocity);
    builder.CheckOk(builder.Send(drivetrain_status_builder.Finish()));
  }

  void TestRollerFront(double roller_speed_front,
                       double roller_speed_compensation) {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_speed_front(roller_speed_front);
    goal_builder.add_roller_speed_compensation(roller_speed_compensation);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
    EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(),
              roller_speed_front + std::max((superstructure_.robot_velocity() *
                                             roller_speed_compensation),
                                            0.0));
    if (superstructure_.robot_velocity() <= 0) {
      EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(),
                roller_speed_front);
    }
  }

  void TestRollerBack(double roller_speed_back,
                      double roller_speed_compensation) {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_speed_back(roller_speed_back);
    goal_builder.add_roller_speed_compensation(roller_speed_compensation);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
    ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
    EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(),
              roller_speed_back - std::min(superstructure_.robot_velocity() *
                                               roller_speed_compensation,
                                           0.0));
    if (superstructure_.robot_velocity() >= 0) {
      EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(),
                roller_speed_back);
    }
  }

  void TestTransferRoller(double transfer_roller_speed,
                          double roller_speed_compensation) {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_transfer_roller_speed(transfer_roller_speed);
    goal_builder.add_roller_speed_compensation(roller_speed_compensation);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
    ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
    EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(),
              transfer_roller_speed);
  }

  std::shared_ptr<const constants::Values> values_;

  const aos::Node *const roborio_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2022::control_loops::superstructure::Superstructure superstructure_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;
  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Sender<DrivetrainStatus> drivetrain_status_sender_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

// Tests that the superstructure does nothing when the goal is to remain
// still.
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);
  superstructure_plant_.intake_front()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.intake_back()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.turret()->InitializePosition(
      constants::Values::kTurretRange().middle());
  superstructure_plant_.catapult()->InitializePosition(
      constants::Values::kCatapultRange().middle());
  superstructure_plant_.climber()->InitializePosition(
      constants::Values::kClimberRange().middle());

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_front = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().middle());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_back = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().middle());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().middle());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().middle());

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_front(intake_offset_front);
    goal_builder.add_intake_back(intake_offset_back);

    goal_builder.add_turret(turret_offset);

    goal_builder.add_climber(climber_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));
  VerifyNearGoal();

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
}

// Tests that loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  // Set a reasonable goal.
  superstructure_plant_.intake_front()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.intake_back()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.turret()->InitializePosition(
      constants::Values::kTurretRange().middle());
  superstructure_plant_.climber()->InitializePosition(
      constants::Values::kClimberRange().middle());
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_front = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_back = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().upper,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_front(intake_offset_front);
    goal_builder.add_intake_back(intake_offset_back);

    goal_builder.add_turret(turret_offset);

    goal_builder.add_climber(climber_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(15));

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
        intake_offset_front = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_back = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().upper);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_front(intake_offset_front);
    goal_builder.add_intake_back(intake_offset_back);

    goal_builder.add_turret(turret_offset);

    goal_builder.add_climber(climber_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_front = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_back = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_front(intake_offset_front);
    goal_builder.add_intake_back(intake_offset_back);

    goal_builder.add_turret(turret_offset);

    goal_builder.add_climber(climber_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.intake_front()->set_peak_velocity(23.0);
  superstructure_plant_.intake_front()->set_peak_acceleration(0.2);
  superstructure_plant_.intake_back()->set_peak_velocity(23.0);
  superstructure_plant_.intake_back()->set_peak_acceleration(0.2);

  superstructure_plant_.turret()->set_peak_velocity(23.0);
  superstructure_plant_.turret()->set_peak_acceleration(6.0);

  superstructure_plant_.climber()->set_peak_velocity(23.0);
  superstructure_plant_.climber()->set_peak_acceleration(0.2);

  // Intake needs over 9 seconds to reach the goal
  // TODO(Milo): Make this a sane time
  RunFor(chrono::seconds(20));
  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.intake_front().state());
  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.intake_back().state());
  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.turret().state());
  EXPECT_EQ(RelativeEncoderSubsystem::State::RUNNING,
            superstructure_.climber().state());
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
}

// Makes sure the roller speeds are getting compensated correctly
TEST_F(SuperstructureTest, RunRollers) {
  SetEnabled(true);
  WaitUntilZeroed();

  SendRobotVelocity(3.0);
  TestRollerFront(-12.0, 1.5);
  TestRollerFront(12.0, 1.5);
  TestRollerFront(0.0, 1.5);

  SendRobotVelocity(-3.0);
  TestRollerFront(-12.0, 1.5);
  TestRollerFront(12.0, 1.5);
  TestRollerFront(0.0, 1.5);

  SendRobotVelocity(3.0);
  TestRollerBack(-12.0, 1.5);
  TestRollerBack(12.0, 1.5);
  TestRollerBack(0.0, 1.5);

  SendRobotVelocity(-3.0);
  TestRollerBack(-12.0, 1.5);
  TestRollerBack(12.0, 1.5);
  TestRollerBack(0.0, 1.5);

  TestTransferRoller(-12.0, 1.5);
  TestTransferRoller(12.0, 1.5);
  TestTransferRoller(0.0, 1.5);
}

// Make sure that the front and back intakes are never switched
TEST_F(SuperstructureTest, RunIntakes) {
  SetEnabled(true);
  superstructure_plant_.intake_front()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.intake_back()->InitializePosition(
      constants::Values::kIntakeRange().middle());

  WaitUntilZeroed();

  superstructure_plant_.intake_front()->set_peak_velocity(23.0);
  superstructure_plant_.intake_front()->set_peak_acceleration(0.3);
  superstructure_plant_.intake_back()->set_peak_velocity(23.0);
  superstructure_plant_.intake_back()->set_peak_acceleration(0.3);

  auto builder = superstructure_goal_sender_.MakeBuilder();
  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_offset_front = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *builder.fbb(), constants::Values::kIntakeRange().lower,
          CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_offset_back = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *builder.fbb(), constants::Values::kIntakeRange().upper,
          CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

  Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

  goal_builder.add_intake_front(intake_offset_front);
  goal_builder.add_intake_back(intake_offset_back);

  ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  // TODO(Milo): Make this a sane time
  RunFor(chrono::seconds(10));
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_FLOAT_EQ(superstructure_status_fetcher_->intake_front()->position(),
                  constants::Values::kIntakeRange().lower);
  EXPECT_FLOAT_EQ(superstructure_status_fetcher_->intake_back()->position(),
                  constants::Values::kIntakeRange().upper);
}

// Make sure that we can shoot the catapult and reload it.
TEST_F(SuperstructureTest, ShootCatapult) {
  SetEnabled(true);
  superstructure_plant_.intake_front()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.intake_back()->InitializePosition(
      constants::Values::kIntakeRange().middle());

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        catapult_return_position_offset =
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *builder.fbb(), constants::Values::kCatapultRange().lower,
                CreateProfileParameters(*builder.fbb(), 4.0, 20.0));

    CatapultGoal::Builder catapult_goal_builder =
        builder.MakeBuilder<CatapultGoal>();

    catapult_goal_builder.add_fire(false);
    catapult_goal_builder.add_shot_position(0.3);
    catapult_goal_builder.add_shot_velocity(15.0);
    catapult_goal_builder.add_return_position(catapult_return_position_offset);
    flatbuffers::Offset<CatapultGoal> catapult_goal_offset =
        catapult_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_catapult(catapult_goal_offset);
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_FALSE(superstructure_status_fetcher_->mpc_active());
  EXPECT_FLOAT_EQ(superstructure_status_fetcher_->catapult()->position(),
                  constants::Values::kCatapultRange().lower);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        catapult_return_position_offset =
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *builder.fbb(), constants::Values::kCatapultRange().lower,
                CreateProfileParameters(*builder.fbb(), 4.0, 20.0));

    CatapultGoal::Builder catapult_goal_builder =
        builder.MakeBuilder<CatapultGoal>();

    catapult_goal_builder.add_fire(true);
    catapult_goal_builder.add_shot_position(0.5);
    catapult_goal_builder.add_shot_velocity(20.0);
    catapult_goal_builder.add_return_position(catapult_return_position_offset);
    flatbuffers::Offset<CatapultGoal> catapult_goal_offset =
        catapult_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_catapult(catapult_goal_offset);
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(100));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_TRUE(superstructure_status_fetcher_->mpc_active());
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  EXPECT_GT(superstructure_status_fetcher_->catapult()->position(),
            constants::Values::kCatapultRange().lower + 0.1);
  RunFor(chrono::milliseconds(1950));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_NEAR(superstructure_status_fetcher_->catapult()->position(),
              constants::Values::kCatapultRange().lower, 1e-3);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 1);
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
