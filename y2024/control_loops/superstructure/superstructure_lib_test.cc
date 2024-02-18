#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/subsystem_simulator.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "y2024/constants/simulated_constants_sender.h"
#include "y2024/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2024/control_loops/superstructure/climber/climber_plant.h"
#include "y2024/control_loops/superstructure/intake_pivot/intake_pivot_plant.h"
#include "y2024/control_loops/superstructure/superstructure.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2024::control_loops::superstructure::testing {

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
typedef Superstructure::AbsoluteEncoderSubsystem AbsoluteEncoderSubsystem;
using PotAndAbsoluteEncoderSimulator =
    frc971::control_loops::SubsystemSimulator<
        frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
        PotAndAbsoluteEncoderSubsystem::State,
        constants::Values::PotAndAbsEncoderConstants>;
using AbsoluteEncoderSimulator = frc971::control_loops::SubsystemSimulator<
    frc971::control_loops::AbsoluteEncoderProfiledJointStatus,
    AbsoluteEncoderSubsystem::State,
    constants::Values::AbsoluteEncoderConstants>;

class SuperstructureSimulation {
 public:
  SuperstructureSimulation(::aos::EventLoop *event_loop,
                           const Constants *simulated_robot_constants,
                           chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")),
        transfer_beambreak_(false),
        intake_pivot_(
            new CappedTestPlant(intake_pivot::MakeIntakePivotPlant()),
            PositionSensorSimulator(simulated_robot_constants->robot()
                                        ->intake_constants()
                                        ->one_revolution_distance()),
            {.subsystem_params =
                 {simulated_robot_constants->common()->intake_pivot(),
                  simulated_robot_constants->robot()->intake_constants()}},
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants->common()->intake_pivot()->range()),
            simulated_robot_constants->robot()
                ->intake_constants()
                ->measured_absolute_position(),
            dt_),
        climber_(new CappedTestPlant(climber::MakeClimberPlant()),
                 PositionSensorSimulator(simulated_robot_constants->robot()
                                             ->climber_constants()
                                             ->zeroing_constants()
                                             ->one_revolution_distance()),
                 {.subsystem_params =
                      {simulated_robot_constants->common()->climber(),
                       simulated_robot_constants->robot()
                           ->climber_constants()
                           ->zeroing_constants()},
                  .potentiometer_offset = simulated_robot_constants->robot()
                                              ->climber_constants()
                                              ->potentiometer_offset()},
                 frc971::constants::Range::FromFlatbuffer(
                     simulated_robot_constants->common()->climber()->range()),
                 simulated_robot_constants->robot()
                     ->climber_constants()
                     ->zeroing_constants()
                     ->measured_absolute_position(),
                 dt_) {
    intake_pivot_.InitializePosition(
        frc971::constants::Range::FromFlatbuffer(
            simulated_robot_constants->common()->intake_pivot()->range())
            .middle());
    climber_.InitializePosition(
        frc971::constants::Range::FromFlatbuffer(
            simulated_robot_constants->common()->climber()->range())
            .middle());
    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
            EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

            intake_pivot_.Simulate(
                superstructure_output_fetcher_->intake_pivot_voltage(),
                superstructure_status_fetcher_->intake_pivot());

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

    frc971::AbsolutePosition::Builder intake_pivot_builder =
        builder.MakeBuilder<frc971::AbsolutePosition>();
    flatbuffers::Offset<frc971::AbsolutePosition> intake_pivot_offset =
        intake_pivot_.encoder()->GetSensorValues(&intake_pivot_builder);

    frc971::PotAndAbsolutePosition::Builder climber_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> climber_offset =
        climber_.encoder()->GetSensorValues(&climber_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_transfer_beambreak(transfer_beambreak_);
    position_builder.add_intake_pivot(intake_pivot_offset);

    position_builder.add_climber(climber_offset);
    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

  void set_transfer_beambreak(bool triggered) {
    transfer_beambreak_ = triggered;
  }

  AbsoluteEncoderSimulator *intake_pivot() { return &intake_pivot_; }

  PotAndAbsoluteEncoderSimulator *climber() { return &climber_; }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool transfer_beambreak_;

  AbsoluteEncoderSimulator intake_pivot_;
  PotAndAbsoluteEncoderSimulator climber_;

  bool first_ = true;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2024/aos_config.json"),
            std::chrono::microseconds(5050)),
        values_(std::make_shared<constants::Values>(constants::MakeValues())),
        simulated_constants_dummy_(SendSimulationConstants(
            event_loop_factory(), 7971, "y2024/constants/test_constants.json")),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        logger_pi_(aos::configuration::GetNode(configuration(), "logger")),
        superstructure_event_loop(MakeEventLoop("Superstructure", roborio_)),
        superstructure_(superstructure_event_loop.get(), (values_)),
        test_event_loop_(MakeEventLoop("test", roborio_)),
        constants_fetcher_(test_event_loop_.get()),
        simulated_robot_constants_(
            CHECK_NOTNULL(&constants_fetcher_.constants())),
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
        superstructure_plant_(superstructure_plant_event_loop_.get(),
                              simulated_robot_constants_, dt()) {
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
    superstructure_output_fetcher_.Fetch();

    ASSERT_FALSE(superstructure_status_fetcher_->estopped());

    ASSERT_TRUE(superstructure_goal_fetcher_.get() != nullptr) << ": No goal";
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr)
        << ": No status";
    ASSERT_TRUE(superstructure_output_fetcher_.get() != nullptr)
        << ": No output";

    double set_point = simulated_robot_constants_->common()
                           ->intake_pivot_set_points()
                           ->retracted();

    if (superstructure_goal_fetcher_->intake_pivot_goal() ==
        IntakePivotGoal::EXTENDED) {
      set_point = simulated_robot_constants_->common()
                      ->intake_pivot_set_points()
                      ->extended();
    }

    EXPECT_NEAR(set_point,
                superstructure_status_fetcher_->intake_pivot()->position(),
                0.001);

    if (superstructure_status_fetcher_->intake_roller() ==
        IntakeRollerStatus::NONE) {
      EXPECT_EQ(superstructure_output_fetcher_->intake_roller_voltage(), 0.0);
    }

    if (superstructure_goal_fetcher_->has_climber_goal()) {
      double set_point =
          simulated_robot_constants_->common()->climber_set_points()->retract();

      if (superstructure_goal_fetcher_->climber_goal() ==
          ClimberGoal::FULL_EXTEND) {
        set_point = simulated_robot_constants_->common()
                        ->climber_set_points()
                        ->full_extend();
      } else if (superstructure_goal_fetcher_->climber_goal() ==
                 ClimberGoal::HALF_EXTEND) {
        set_point = simulated_robot_constants_->common()
                        ->climber_set_points()
                        ->half_extend();
      }

      EXPECT_NEAR(set_point,
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

  std::shared_ptr<const constants::Values> values_;
  const bool simulated_constants_dummy_;

  const aos::Node *const roborio_;
  const aos::Node *const logger_pi_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2024::control_loops::superstructure::Superstructure superstructure_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const Constants *simulated_robot_constants_;

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

  const ::std::vector<::Eigen::Matrix<double, 3, 1>> points_;
};

// Tests that the superstructure does nothing when the goal is to remain
// still.

TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_pivot_goal(IntakePivotGoal::RETRACTED);
    goal_builder.add_climber_goal(ClimberGoal::RETRACT);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());

  VerifyNearGoal();
}

// Tests that loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  superstructure_plant_.intake_pivot()->InitializePosition(
      frc971::constants::Range::FromFlatbuffer(
          simulated_robot_constants_->common()->intake_pivot()->range())
          .middle());
  superstructure_plant_.climber()->InitializePosition(
      frc971::constants::Range::FromFlatbuffer(
          simulated_robot_constants_->common()->climber()->range())
          .lower);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_pivot_goal(IntakePivotGoal::EXTENDED);
    goal_builder.add_climber_goal(ClimberGoal::FULL_EXTEND);

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

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_pivot_goal(IntakePivotGoal::EXTENDED);
    goal_builder.add_climber_goal(ClimberGoal::FULL_EXTEND);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(20));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_pivot_goal(IntakePivotGoal::RETRACTED);
    goal_builder.add_climber_goal(ClimberGoal::RETRACT);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));
  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));

  EXPECT_EQ(AbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.intake_pivot().state());

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.climber().state());
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
}

// Tests Climber in multiple scenarios
TEST_F(SuperstructureTest, ClimberTest) {
  SetEnabled(true);
  WaitUntilZeroed();

  superstructure_plant_.climber()->InitializePosition(
      frc971::constants::Range::FromFlatbuffer(
          simulated_robot_constants_->common()->climber()->range())
          .middle());

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_goal(ClimberGoal::FULL_EXTEND);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_goal(ClimberGoal::HALF_EXTEND);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_goal(ClimberGoal::RETRACT);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests intake and transfer in multiple scenarios
TEST_F(SuperstructureTest, IntakeGoal) {
  SetEnabled(true);
  WaitUntilZeroed();

  superstructure_plant_.intake_pivot()->InitializePosition(
      frc971::constants::Range::FromFlatbuffer(
          simulated_robot_constants_->common()->intake_pivot()->range())
          .middle());

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_pivot_goal(IntakePivotGoal::RETRACTED);
    goal_builder.add_intake_roller_goal(IntakeRollerGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_transfer_beambreak(false);

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_pivot_goal(IntakePivotGoal::EXTENDED);
    goal_builder.add_intake_roller_goal(IntakeRollerGoal::SPIT);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_transfer_beambreak(false);

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(),
            simulated_robot_constants_->common()
                ->transfer_roller_voltages()
                ->transfer_out());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_pivot_goal(IntakePivotGoal::EXTENDED);
    goal_builder.add_intake_roller_goal(IntakeRollerGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_transfer_beambreak(false);

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_roller_goal(IntakeRollerGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_transfer_beambreak(false);

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(),
            simulated_robot_constants_->common()
                ->transfer_roller_voltages()
                ->transfer_in());

  superstructure_plant_.set_transfer_beambreak(true);

  RunFor(chrono::seconds(2));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);
}
}  // namespace y2024::control_loops::superstructure::testing
