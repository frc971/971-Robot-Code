#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/subsystem_simulator.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2022_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022_bot3/control_loops/superstructure/superstructure.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2022_bot3 {
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
using DrivetrainStatus = ::frc971::control_loops::drivetrain::Status;
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;
using PotAndAbsoluteEncoderSimulator =
    frc971::control_loops::SubsystemSimulator<
        frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
        PotAndAbsoluteEncoderSubsystem::State,
        constants::Values::PotAndAbsEncoderConstants>;

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
        intake_(new CappedTestPlant(intake::MakeIntakePlant()),
                PositionSensorSimulator(
                    values->intake.subsystem_params.zeroing_constants
                        .one_revolution_distance),
                values->intake, constants::Values::kIntakeRange(),
                values->intake.subsystem_params.zeroing_constants
                    .measured_absolute_position,
                dt_),
        climber_right_(
            new CappedTestPlant(climber::MakeClimberPlant()),
            PositionSensorSimulator(
                constants::Values::kClimberEncoderMetersPerRevolution()),
            values->climber_right, constants::Values::kClimberRange(),
            values->climber_right.subsystem_params.zeroing_constants
                .measured_absolute_position,
            dt_),
        climber_left_(
            new CappedTestPlant(climber::MakeClimberPlant()),
            PositionSensorSimulator(
                constants::Values::kClimberEncoderMetersPerRevolution()),
            values->climber_left, constants::Values::kClimberRange(),
            values->climber_left.subsystem_params.zeroing_constants
                .measured_absolute_position,
            dt_) {
    intake_.InitializePosition(constants::Values::kIntakeRange().middle());
    climber_left_.InitializePosition(
        constants::Values::kClimberRange().middle());
    climber_right_.InitializePosition(
        constants::Values::kClimberRange().middle());

    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time
          if (!first_) {
            ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
            ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

            intake_.Simulate(superstructure_output_fetcher_->intake_voltage(),
                             superstructure_status_fetcher_->intake());
            climber_left_.Simulate(
                superstructure_output_fetcher_->climber_voltage_left(),
                superstructure_status_fetcher_->climber_left());
            climber_right_.Simulate(
                superstructure_output_fetcher_->climber_voltage_right(),
                superstructure_status_fetcher_->climber_right());
          } else {
            first_ = false;
          }
          SendPositionMessage();
        },
        dt);
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::PotAndAbsolutePosition::Builder intake_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_offset =
        intake_.encoder()->GetSensorValues(&intake_builder);

    frc971::PotAndAbsolutePosition::Builder climber_right_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> climber_right_offset =
        climber_right_.encoder()->GetSensorValues(&climber_right_builder);

    frc971::PotAndAbsolutePosition::Builder climber_left_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> climber_left_offset =
        climber_left_.encoder()->GetSensorValues(&climber_left_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_intake(intake_offset);
    position_builder.add_climber_right(climber_right_offset);
    position_builder.add_climber_left(climber_left_offset);

    builder.CheckOk(builder.Send(position_builder.Finish()));
  }

  PotAndAbsoluteEncoderSimulator *intake() { return &intake_; }
  PotAndAbsoluteEncoderSimulator *climber_left() { return &climber_left_; }
  PotAndAbsoluteEncoderSimulator *climber_right() { return &climber_right_; }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool first_ = true;

  PotAndAbsoluteEncoderSimulator intake_;
  PotAndAbsoluteEncoderSimulator climber_right_;
  PotAndAbsoluteEncoderSimulator climber_left_;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2022_bot3/aos_config.json"),
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
  }

  void VerifyNearGoal() {
    superstructure_status_fetcher_.Fetch();
    superstructure_goal_fetcher_.Fetch();

    if (superstructure_goal_fetcher_->has_intake()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->intake()->unsafe_goal(),
                  superstructure_status_fetcher_->intake()->position(), 0.001);
    }

    if (superstructure_goal_fetcher_->has_climber_left()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->climber_left()->unsafe_goal(),
                  superstructure_status_fetcher_->climber_left()->position(),
                  0.001);
    }

    if (superstructure_goal_fetcher_->has_climber_right()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->climber_right()->unsafe_goal(),
                  superstructure_status_fetcher_->climber_right()->position(),
                  0.001);
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

  void TestRoller(double roller_speed) {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_speed(roller_speed);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
    EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), roller_speed);
  }

  std::shared_ptr<const constants::Values> values_;

  const aos::Node *const roborio_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2022_bot3::control_loops::superstructure::Superstructure superstructure_;
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

};  // namespace testing

// Tests that the superstructure does nothing when the goal is to remain still
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);

  superstructure_plant_.intake()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.climber_left()->InitializePosition(
      constants::Values::kClimberRange().middle());
  superstructure_plant_.climber_right()->InitializePosition(
      constants::Values::kClimberRange().middle());

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().middle());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_right_offset =
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *builder.fbb(), constants::Values::kClimberRange().middle());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_left_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().middle());

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake(intake_offset);
    goal_builder.add_climber_right(climber_right_offset);
    goal_builder.add_climber_left(climber_left_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));
  VerifyNearGoal();

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
}

// Tests that the loop can reach a goal
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);

  // Set initial position of climber, intake
  superstructure_plant_.intake()->InitializePosition(
      constants::Values::kIntakeRange().middle());
  superstructure_plant_.climber_left()->InitializePosition(
      values_->kClimberRange().middle());
  superstructure_plant_.climber_right()->InitializePosition(
      values_->kClimberRange().middle());

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_left_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().lower,
            CreateProfileParameters(*builder.fbb(), 1.0, 0.2));
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_right_offset =
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *builder.fbb(), constants::Values::kClimberRange().lower,
                CreateProfileParameters(*builder.fbb(), 1.0, 0.2));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake(intake_offset);
    goal_builder.add_climber_left(climber_left_offset);
    goal_builder.add_climber_right(climber_right_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Give it enough time to get to the goal
  RunFor(chrono::seconds(15));

  VerifyNearGoal();
}

// Make sure the motor voltage is pulled back to prevent oscillation
TEST_F(SuperstructureTest, SaturationTest) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset_left = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset_right =
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *builder.fbb(), constants::Values::kClimberRange().upper);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake(intake_offset);

    goal_builder.add_climber_left(climber_offset_left);
    goal_builder.add_climber_right(climber_offset_right);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(20));
  VerifyNearGoal();

  // Test a low acceleartion move with a high velocity
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kIntakeRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset_left = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kClimberRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        climber_offset_right =
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *builder.fbb(), constants::Values::kClimberRange().lower,
                CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake(intake_offset);

    goal_builder.add_climber_left(climber_offset_left);

    goal_builder.add_climber_right(climber_offset_right);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Values here are min/max velocity/acceleration
  superstructure_plant_.intake()->set_peak_velocity(23.0);
  superstructure_plant_.intake()->set_peak_acceleration(0.2);

  superstructure_plant_.climber_left()->set_peak_velocity(23.0);
  superstructure_plant_.climber_left()->set_peak_acceleration(0.2);
  superstructure_plant_.climber_right()->set_peak_velocity(23.0);
  superstructure_plant_.climber_right()->set_peak_acceleration(0.2);

  RunFor(chrono::seconds(20));
  VerifyNearGoal();
}

// Tests that the loop zeroes when run without a goal
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.intake().state());
  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.climber_left().state());
  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.climber_right().state());
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
}

TEST_F(SuperstructureTest, RunRollers) {
  TestRoller(-12.0);
  TestRoller(12.0);
  TestRoller(0.0);
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022_bot3