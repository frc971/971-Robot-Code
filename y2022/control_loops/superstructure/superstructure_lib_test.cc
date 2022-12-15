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
#include "frc971/control_loops/subsystem_simulator.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace testing {
namespace chrono = std::chrono;

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
using PotAndAbsoluteEncoderSimulator = frc971::control_loops::SubsystemSimulator<
    frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
    PotAndAbsoluteEncoderSubsystem::State,
    constants::Values::PotAndAbsEncoderConstants>;
using RelativeEncoderSimulator = frc971::control_loops::SubsystemSimulator<
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
                values->turret, values->turret_range,
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
    turret_.InitializePosition(values->turret_range.middle());
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
            if (superstructure_status_fetcher_->mpc_horizon()) {
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

    frc971::RelativePosition::Builder flipper_arm_left_builder =
        builder.MakeBuilder<frc971::RelativePosition>();
    flipper_arm_left_builder.add_encoder(flipper_arm_left_);
    flatbuffers::Offset<frc971::RelativePosition> flipper_arm_left_offset =
        flipper_arm_left_builder.Finish();

    frc971::RelativePosition::Builder flipper_arm_right_builder =
        builder.MakeBuilder<frc971::RelativePosition>();
    flipper_arm_right_builder.add_encoder(flipper_arm_right_);
    flatbuffers::Offset<frc971::RelativePosition> flipper_arm_right_offset =
        flipper_arm_left_builder.Finish();

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_intake_front(intake_front_offset);
    position_builder.add_intake_back(intake_back_offset);
    position_builder.add_turret(turret_offset);
    position_builder.add_catapult(catapult_offset);
    position_builder.add_climber(climber_offset);
    position_builder.add_intake_beambreak_front(intake_beambreak_front_);
    position_builder.add_intake_beambreak_back(intake_beambreak_back_);
    position_builder.add_turret_beambreak(turret_beambreak_);
    position_builder.add_flipper_arm_left(flipper_arm_left_offset);
    position_builder.add_flipper_arm_right(flipper_arm_right_offset);

    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

  PotAndAbsoluteEncoderSimulator *intake_front() { return &intake_front_; }
  PotAndAbsoluteEncoderSimulator *intake_back() { return &intake_back_; }
  PotAndAbsoluteEncoderSimulator *turret() { return &turret_; }
  PotAndAbsoluteEncoderSimulator *catapult() { return &catapult_; }
  RelativeEncoderSimulator *climber() { return &climber_; }

  void set_intake_beambreak_front(bool triggered) {
    intake_beambreak_front_ = triggered;
  }

  void set_intake_beambreak_back(bool triggered) {
    intake_beambreak_back_ = triggered;
  }

  void set_turret_beambreak(bool triggered) { turret_beambreak_ = triggered; }

  void set_flipper_arm_left(double pos) { flipper_arm_left_ = pos; }
  void set_flipper_arm_right(double pos) { flipper_arm_right_ = pos; }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool first_ = true;

  bool intake_beambreak_front_ = false;
  bool intake_beambreak_back_ = false;
  bool turret_beambreak_ = false;
  double flipper_arm_left_ = 0.0;
  double flipper_arm_right_ = 0.0;
  PotAndAbsoluteEncoderSimulator intake_front_;
  PotAndAbsoluteEncoderSimulator intake_back_;
  PotAndAbsoluteEncoderSimulator turret_;
  PotAndAbsoluteEncoderSimulator catapult_;
  RelativeEncoderSimulator climber_;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  static constexpr double kSafeTurretAngle =
      CollisionAvoidance::kMaxCollisionZoneBackTurret +
      CollisionAvoidance::kEpsTurret;

  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2022/aos_config.json"),
            std::chrono::microseconds(5050)),
        values_(std::make_shared<constants::Values>(constants::MakeValues())),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        logger_pi_(aos::configuration::GetNode(configuration(), "logger")),
        superstructure_event_loop(MakeEventLoop("Superstructure", roborio_)),
        superstructure_(superstructure_event_loop.get(), values_),
        test_event_loop_(MakeEventLoop("test", roborio_)),
        ball_color_event_loop_(MakeEventLoop("ball color test", logger_pi_)),
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
        ball_color_sender_(
            ball_color_event_loop_->MakeSender<y2022::vision::BallColor>(
                "/superstructure")),
        superstructure_plant_event_loop_(MakeEventLoop("plant", roborio_)),
        superstructure_plant_(superstructure_plant_event_loop_.get(), values_,
                              dt()) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);

    SetEnabled(true);

    if (!FLAGS_output_folder.empty()) {
      unlink(FLAGS_output_folder.c_str());
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_folder);
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

    if (superstructure_status_fetcher_->intake_state() !=
        IntakeState::NO_BALL) {
      EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 0.0);
      EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 0.0);
    }

    EXPECT_NEAR(superstructure_goal_fetcher_->climber()->unsafe_goal(),
                superstructure_status_fetcher_->climber()->position(), 0.001);

    if (superstructure_goal_fetcher_->has_turret() &&
        superstructure_status_fetcher_->state() !=
            SuperstructureState::TRANSFERRING) {
      EXPECT_NEAR(superstructure_goal_fetcher_->turret()->unsafe_goal(),
                  superstructure_status_fetcher_->turret()->position(), 0.001);
    }
  }  // namespace testing

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
    SendDrivetrainStatus(robot_velocity, {0.0, 0.0}, 0.0);
  }

  void SendDrivetrainStatus(double robot_velocity, Eigen::Vector2d pos,
                            double theta) {
    // Send a robot velocity to test compensation
    auto builder = drivetrain_status_sender_.MakeBuilder();
    auto drivetrain_status_builder = builder.MakeBuilder<DrivetrainStatus>();
    drivetrain_status_builder.add_robot_speed(robot_velocity);
    drivetrain_status_builder.add_estimated_left_velocity(robot_velocity);
    drivetrain_status_builder.add_estimated_right_velocity(robot_velocity);
    drivetrain_status_builder.add_x(pos.x());
    drivetrain_status_builder.add_y(pos.y());
    drivetrain_status_builder.add_theta(theta);
    builder.CheckOk(builder.Send(drivetrain_status_builder.Finish()));
  }

  void TestRollerFront(double roller_speed_front,
                       double roller_speed_compensation, double expected) {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_speed_front(roller_speed_front);
    goal_builder.add_roller_speed_compensation(roller_speed_compensation);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
    EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), expected);
  }

  void TestRollerBack(double roller_speed_back,
                      double roller_speed_compensation, double expected) {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_speed_back(roller_speed_back);
    goal_builder.add_roller_speed_compensation(roller_speed_compensation);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
    ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

    EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), expected);
  }

  std::shared_ptr<const constants::Values> values_;

  const aos::Node *const roborio_;
  const aos::Node *const logger_pi_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2022::control_loops::superstructure::Superstructure superstructure_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::std::unique_ptr<aos::EventLoop> ball_color_event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;
  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Sender<DrivetrainStatus> drivetrain_status_sender_;
  ::aos::Sender<y2022::vision::BallColor> ball_color_sender_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
};  // namespace testing

// Tests that the superstructure does nothing when the goal is to remain
// still.
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);
  superstructure_plant_.intake_front()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.intake_back()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.turret()->InitializePosition(kSafeTurretAngle);
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
            *builder.fbb(), kSafeTurretAngle);

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
      values_->turret_range.middle());
  superstructure_plant_.climber()->InitializePosition(
      constants::Values::kClimberRange().middle());
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_front = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset_back = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), values_->turret_range.lower,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().lower,
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
  superstructure_plant_.turret()->InitializePosition(kSafeTurretAngle);

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

    // Keep the turret away from the intakes because they start in the collision
    // area
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kSafeTurretAngle);

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
  RunFor(chrono::seconds(20));
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
            *builder.fbb(), values_->turret_range.lower,
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
  TestRollerFront(-12.0, 1.5, -7.5);
  TestRollerFront(12.0, 1.5, 16.5);
  TestRollerFront(0.0, 1.5, 4.5);

  SendRobotVelocity(-3.0);
  TestRollerFront(-12.0, 1.5, -12.0);
  TestRollerFront(12.0, 1.5, 12.0);
  TestRollerFront(0.0, 1.5, 0.0);

  SendRobotVelocity(3.0);
  TestRollerBack(-12.0, 1.5, -12.0);
  TestRollerBack(12.0, 1.5, 12.0);
  TestRollerBack(0.0, 1.5, 0.0);

  SendRobotVelocity(-3.0);
  TestRollerBack(-12.0, 1.5, -7.5);
  TestRollerBack(12.0, 1.5, 16.5);
  TestRollerBack(0.0, 1.5, 4.5);
}

// Tests the whole shooting statemachine - from loading to shooting
TEST_F(SuperstructureTest, LoadingToShooting) {
  SetEnabled(true);
  WaitUntilZeroed();

  SendRobotVelocity(1.0);

  constexpr double kTurretGoal = 2.0;
  constexpr double kCatapultReturnPosition = -0.87;
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kTurretGoal);
    const auto catapult_return_offset =
        CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kCatapultReturnPosition);
    auto catapult_builder = builder.MakeBuilder<CatapultGoal>();
    catapult_builder.add_return_position(catapult_return_offset);
    const auto catapult_offset = catapult_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_speed_front(12.0);
    goal_builder.add_roller_speed_back(12.0);
    goal_builder.add_roller_speed_compensation(0.0);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_turret_intake(RequestedIntake::kFront);
    goal_builder.add_catapult(catapult_offset);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }
  RunFor(std::chrono::seconds(2));

  // Make sure that the rollers are spinning, but the superstructure hasn't
  // transitioned away from idle because the beambreaks haven't been triggered.
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 12.0);

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::NO_BALL);
  // Since we requested the front intake, the turret should be trying to intake
  // from there.
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(),
              constants::Values::kTurretFrontIntakePos(), 0.001);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  superstructure_plant_.set_intake_beambreak_front(true);
  superstructure_plant_.set_intake_beambreak_back(false);
  RunFor(dt());

  // Make sure that the turret goal is set to be loading from the front intake
  // and the supersturcture is transferring from the front intake, since that
  // beambreak was trigerred. Also, the front outside rollers should be stopped.
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::TRANSFERRING);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::INTAKE_FRONT_BALL);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 0.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 12.0);

  RunFor(chrono::seconds(2));

  // Make sure that we are still transferring and the front transfer rollers
  // still have a ball. The turret should now be at the loading position and the
  // flippers should be feeding the ball.
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 0.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 12.0);
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::TRANSFERRING);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::INTAKE_FRONT_BALL);
  EXPECT_EQ(superstructure_output_fetcher_->flipper_arms_voltage(),
            constants::Values::kFlipperFeedVoltage());
  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(),
            constants::Values::kTransferRollerVoltage());
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(),
              constants::Values::kTurretFrontIntakePos(), 0.001);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  superstructure_plant_.set_intake_beambreak_front(false);
  superstructure_plant_.set_intake_beambreak_back(false);
  superstructure_plant_.set_turret_beambreak(true);
  RunFor(dt() * 2);

  // Now that the turret beambreak has been triggered, we should be loading the
  // ball. The outside rollers shouldn't be limited anymore, and the transfer
  // rollers should be off. The flippers should still be feeding the ball, and
  // the intake state should reflect that the ball has been transferred away
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADING);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::NO_BALL);
  EXPECT_EQ(superstructure_output_fetcher_->flipper_arms_voltage(),
            constants::Values::kFlipperFeedVoltage());
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  superstructure_plant_.set_turret_beambreak(false);
  RunFor(constants::Values::kExtraLoadingTime() + dt() * 2);

  // Now that the ball has gone past the turret beambreak,
  // it should be loaded in the catapult and ready for firing.
  // The flippers should be off.
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::NO_BALL);
  EXPECT_EQ(superstructure_output_fetcher_->flipper_arms_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  RunFor(std::chrono::seconds(2));

  // After a few seconds, the turret should be at it's aiming goal. The flippers
  // should still be off and we should still be loaded and ready to fire.
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::NO_BALL);
  EXPECT_EQ(superstructure_output_fetcher_->flipper_arms_voltage(), 0.0);
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(), kTurretGoal,
              0.001);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  superstructure_plant_.set_intake_beambreak_front(false);
  superstructure_plant_.set_intake_beambreak_back(true);
  RunFor(dt() * 2);

  // A ball being intaked from the back should be held by wiggling the transfer
  // rollers, but we shound't abort the shot from the front intake for it and
  // move the turret.
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 12.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 0.0);
  EXPECT_TRUE(superstructure_output_fetcher_->transfer_roller_voltage() !=
                  0.0 &&
              superstructure_output_fetcher_->transfer_roller_voltage() <=
                  constants::Values::kTransferRollerWiggleVoltage() &&
              superstructure_output_fetcher_->transfer_roller_voltage() >=
                  -constants::Values::kTransferRollerWiggleVoltage());
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::INTAKE_BACK_BALL);
  EXPECT_FALSE(superstructure_status_fetcher_->front_intake_has_ball());
  EXPECT_TRUE(superstructure_status_fetcher_->back_intake_has_ball());
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(), kTurretGoal,
              0.001);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kTurretGoal);

    const auto catapult_return_offset =
        CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kCatapultReturnPosition);
    auto catapult_builder = builder.MakeBuilder<CatapultGoal>();
    catapult_builder.add_shot_position(0.3);
    catapult_builder.add_shot_velocity(15.0);
    catapult_builder.add_return_position(catapult_return_offset);
    auto catapult_offset = catapult_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_speed_front(0.0);
    goal_builder.add_roller_speed_back(12.0);
    goal_builder.add_roller_speed_compensation(0.0);
    goal_builder.add_catapult(catapult_offset);
    goal_builder.add_fire(true);
    goal_builder.add_turret(turret_offset);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_flipper_arm_left(
      constants::Values::kFlipperArmRange().upper);
  superstructure_plant_.set_flipper_arm_right(
      constants::Values::kFlipperArmRange().upper);
  RunFor(dt() * 2);

  // Now that we were asked to fire and the flippers are open,
  // we should be shooting the ball and holding the flippers open.
  // The turret should still be at its goal, and we should still be wiggling the
  // transfer rollers to keep the ball in the back intake.
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_front(), 0.0);
  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage_back(), 0.0);
  EXPECT_TRUE(superstructure_output_fetcher_->transfer_roller_voltage() !=
                  0.0 &&
              superstructure_output_fetcher_->transfer_roller_voltage() <=
                  constants::Values::kTransferRollerWiggleVoltage() &&
              superstructure_output_fetcher_->transfer_roller_voltage() >=
                  -constants::Values::kTransferRollerWiggleVoltage());
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::SHOOTING);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::INTAKE_BACK_BALL);
  EXPECT_TRUE(superstructure_status_fetcher_->flippers_open());
  EXPECT_EQ(superstructure_output_fetcher_->flipper_arms_voltage(),
            constants::Values::kFlipperHoldVoltage());
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(), kTurretGoal,
              0.001);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  superstructure_plant_.set_flipper_arm_left(
      constants::Values::kFlipperArmRange().upper);
  superstructure_plant_.set_flipper_arm_right(
      constants::Values::kFlipperArmRange().upper);
  superstructure_plant_.set_intake_beambreak_back(false);
  RunFor(std::chrono::seconds(2));

  // After a bit, we should have completed the shot and be transferring.
  // Since the beambreak was triggered a bit ago, it should still think a ball
  // is there.
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 1);
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::TRANSFERRING);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::INTAKE_BACK_BALL);
  EXPECT_TRUE(superstructure_status_fetcher_->transitioning_second_ball());
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(),
              -constants::Values::kTurretBackIntakePos(), 0.001);

  // Since the intake beambreak hasn't triggered in a while, it should realize
  // the ball was lost.
  RunFor(std::chrono::seconds(1));
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 1);
  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);
  EXPECT_FALSE(superstructure_status_fetcher_->transitioning_second_ball());
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::NO_BALL);
}

TEST_F(SuperstructureTest, TestTurretUnWrapsWhenLoading) {
  SetEnabled(true);
  WaitUntilZeroed();

  constexpr double kTurretGoal = -6.0;
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kTurretGoal);
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_turret(turret_offset);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }
  RunFor(std::chrono::seconds(5));
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::NO_BALL);
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(), kTurretGoal,
              0.001);

  superstructure_plant_.set_intake_beambreak_back(true);
  RunFor(dt() * 2);

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::TRANSFERRING);
  EXPECT_EQ(superstructure_status_fetcher_->intake_state(),
            IntakeState::INTAKE_BACK_BALL);

  RunFor(std::chrono::seconds(3));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_NEAR(superstructure_status_fetcher_->turret()->position(),
              constants::Values::kTurretBackIntakePos(), 0.001);
  // It goes to -pi instead of +pi because -pi is closest to the center of the
  // range at -1.675.
}

// Make sure that the front and back intakes are never switched
TEST_F(SuperstructureTest, RunIntakes) {
  SetEnabled(true);
  superstructure_plant_.intake_front()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.intake_back()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.turret()->InitializePosition(kSafeTurretAngle);

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

  // Keep the turret away from the intakes to not trigger collision avoidance
  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *builder.fbb(), kSafeTurretAngle,
          CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

  Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

  goal_builder.add_intake_front(intake_offset_front);
  goal_builder.add_intake_back(intake_offset_back);
  goal_builder.add_turret(turret_offset);

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
      constants::Values::kIntakeRange().lower);
  superstructure_plant_.intake_back()->InitializePosition(
      constants::Values::kIntakeRange().lower);
  superstructure_plant_.turret()->InitializePosition(
      constants::Values::kTurretFrontIntakePos());

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

    catapult_goal_builder.add_shot_position(0.3);
    catapult_goal_builder.add_shot_velocity(15.0);
    catapult_goal_builder.add_return_position(catapult_return_position_offset);
    flatbuffers::Offset<CatapultGoal> catapult_goal_offset =
        catapult_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_fire(false);
    goal_builder.add_catapult(catapult_goal_offset);
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->mpc_horizon(), 0u);
  EXPECT_FLOAT_EQ(superstructure_status_fetcher_->catapult()->position(),
                  constants::Values::kCatapultRange().lower);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_goal_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kTurretFrontIntakePos());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        catapult_return_position_offset =
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *builder.fbb(), constants::Values::kCatapultRange().lower,
                CreateProfileParameters(*builder.fbb(), 4.0, 20.0));

    CatapultGoal::Builder catapult_goal_builder =
        builder.MakeBuilder<CatapultGoal>();

    catapult_goal_builder.add_shot_position(0.5);
    catapult_goal_builder.add_shot_velocity(20.0);
    catapult_goal_builder.add_return_position(catapult_return_position_offset);
    flatbuffers::Offset<CatapultGoal> catapult_goal_offset =
        catapult_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_fire(true);
    goal_builder.add_catapult(catapult_goal_offset);
    goal_builder.add_turret(turret_goal_offset);
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Make the superstructure statemachine progress to SHOOTING
  superstructure_plant_.set_intake_beambreak_front(true);
  superstructure_plant_.set_turret_beambreak(true);
  superstructure_plant_.set_flipper_arm_left(
      constants::Values::kFlipperArmRange().upper);
  superstructure_plant_.set_flipper_arm_right(
      constants::Values::kFlipperArmRange().upper);

  RunFor(dt() * 4);

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADING);
  superstructure_plant_.set_turret_beambreak(false);

  RunFor(chrono::milliseconds(200));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_NE(superstructure_status_fetcher_->mpc_horizon(), 0u);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 0);

  EXPECT_GT(superstructure_status_fetcher_->catapult()->position(),
            constants::Values::kCatapultRange().lower + 0.1);
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::SHOOTING);
  superstructure_plant_.set_intake_beambreak_front(false);

  RunFor(chrono::milliseconds(1950));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_NEAR(superstructure_status_fetcher_->catapult()->position(),
              constants::Values::kCatapultRange().lower, 1e-3);
  EXPECT_EQ(superstructure_status_fetcher_->shot_count(), 1);
  // Will still be in TRANSFERRING because we left the front beambreak on after
  // we entered loaded, so we queued up another ball.
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::TRANSFERRING);

  RunFor(chrono::milliseconds(2000));
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);
}

// Test that we are able to signal that the ball was preloaded
TEST_F(SuperstructureTest, Preloaded) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_preloaded(true);
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(dt());

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);
}

// Tests that the turret switches to auto-aiming when we set auto_aim to
// true.
TEST_F(SuperstructureTest, TurretAutoAim) {
  SetEnabled(true);
  WaitUntilZeroed();

  // Set ourselves up 5m from the target--the turret goal should be 90 deg (we
  // need to shoot out the right of the robot, and we shoot out of the back of
  // the turret).
  SendDrivetrainStatus(0.0, {0.0, 5.0}, 0.0);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_auto_aim(true);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Give it time to stabilize.
  RunFor(chrono::seconds(2));

  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(M_PI_2, superstructure_status_fetcher_->turret()->position(),
              5e-4);
  EXPECT_FLOAT_EQ(M_PI_2,
                  superstructure_status_fetcher_->aimer()->turret_position());
  EXPECT_FLOAT_EQ(0,
                  superstructure_status_fetcher_->aimer()->turret_velocity());
}

TEST_F(SuperstructureTest, InterpolationTableTest) {
  SetEnabled(true);
  WaitUntilZeroed();

  constexpr double kDistance = 3.0;

  SendDrivetrainStatus(0.0, {0.0, kDistance}, 0.0);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_auto_aim(true);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Give it time to stabilize.
  RunFor(chrono::seconds(2));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_NEAR(superstructure_status_fetcher_->aimer()->target_distance(),
              kDistance, 0.01);

  constants::Values::ShotParams shot_params;
  EXPECT_TRUE(
      values_->shot_interpolation_table.GetInRange(kDistance, &shot_params));

  EXPECT_EQ(superstructure_status_fetcher_->shot_velocity(),
            shot_params.shot_velocity);
  EXPECT_EQ(superstructure_status_fetcher_->shot_position(),
            shot_params.shot_angle);
}

// Tests that balls get discarded when they are the wrong color.
TEST_F(SuperstructureTest, BallDiscarding) {
  set_alliance(aos::Alliance::kInvalid);
  SetEnabled(true);
  WaitUntilZeroed();

  // Set ourselves up 5m from the target--the turret goal should be 90 deg (we
  // need to shoot out the right of the robot, and we shoot out of the back of
  // the turret).
  SendDrivetrainStatus(0.0, {0.0, 5.0}, 0.0);

  RunFor(chrono::milliseconds(500));
  set_alliance(aos::Alliance::kBlue);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_auto_aim(true);
    goal_builder.add_preloaded(true);
    goal_builder.add_turret_intake(RequestedIntake::kFront);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Give it time to stabilize.
  RunFor(chrono::seconds(2));

  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(M_PI_2, superstructure_status_fetcher_->turret()->position(),
              5e-4);

  {
    auto builder = ball_color_sender_.MakeBuilder();

    y2022::vision::BallColor::Builder ball_color_builder =
        builder.MakeBuilder<y2022::vision::BallColor>();

    ball_color_builder.add_ball_color(aos::Alliance::kBlue);

    ASSERT_EQ(builder.Send(ball_color_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(100));
  superstructure_status_fetcher_.Fetch();
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);

  {
    auto builder = ball_color_sender_.MakeBuilder();

    y2022::vision::BallColor::Builder ball_color_builder =
        builder.MakeBuilder<y2022::vision::BallColor>();

    ball_color_builder.add_ball_color(aos::Alliance::kRed);

    ASSERT_EQ(builder.Send(ball_color_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(dt());

  superstructure_status_fetcher_.Fetch();
  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::SHOOTING);

  RunFor(chrono::milliseconds(2000));
  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(constants::Values::kTurretFrontIntakePos(),
              superstructure_status_fetcher_->turret()->position(), 5e-4);
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
