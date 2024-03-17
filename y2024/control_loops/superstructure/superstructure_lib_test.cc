#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/catapult/catapult_goal_static.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/subsystem_simulator.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "y2024/constants/simulated_constants_sender.h"
#include "y2024/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2024/control_loops/superstructure/altitude/altitude_plant.h"
#include "y2024/control_loops/superstructure/catapult/catapult_plant.h"
#include "y2024/control_loops/superstructure/climber/climber_plant.h"
#include "y2024/control_loops/superstructure/extend/extend_plant.h"
#include "y2024/control_loops/superstructure/intake_pivot/intake_pivot_plant.h"
#include "y2024/control_loops/superstructure/superstructure.h"
#include "y2024/control_loops/superstructure/turret/turret_plant.h"

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
        superstructure_can_position_sender_(
            event_loop_->MakeSender<CANPosition>("/superstructure/rio")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")),
        extend_beambreak_(false),
        catapult_beambreak_(false),
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
                 dt_),
        catapult_(new CappedTestPlant(catapult::MakeCatapultPlant()),
                  PositionSensorSimulator(simulated_robot_constants->robot()
                                              ->catapult_constants()
                                              ->zeroing_constants()
                                              ->one_revolution_distance()),
                  {.subsystem_params =
                       {simulated_robot_constants->common()->catapult(),
                        simulated_robot_constants->robot()
                            ->catapult_constants()
                            ->zeroing_constants()},
                   .potentiometer_offset = simulated_robot_constants->robot()
                                               ->catapult_constants()
                                               ->potentiometer_offset()},
                  frc971::constants::Range::FromFlatbuffer(
                      simulated_robot_constants->common()->catapult()->range()),
                  simulated_robot_constants->robot()
                      ->catapult_constants()
                      ->zeroing_constants()
                      ->measured_absolute_position(),
                  dt_),
        altitude_(new CappedTestPlant(altitude::MakeAltitudePlant()),
                  PositionSensorSimulator(simulated_robot_constants->robot()
                                              ->altitude_constants()
                                              ->zeroing_constants()
                                              ->one_revolution_distance()),
                  {.subsystem_params =
                       {simulated_robot_constants->common()->altitude(),
                        simulated_robot_constants->robot()
                            ->altitude_constants()
                            ->zeroing_constants()},
                   .potentiometer_offset = simulated_robot_constants->robot()
                                               ->altitude_constants()
                                               ->potentiometer_offset()},
                  frc971::constants::Range::FromFlatbuffer(
                      simulated_robot_constants->common()->altitude()->range()),
                  simulated_robot_constants->robot()
                      ->altitude_constants()
                      ->zeroing_constants()
                      ->measured_absolute_position(),
                  dt_),
        turret_(
            new CappedTestPlant(turret::MakeTurretPlant()),
            PositionSensorSimulator(simulated_robot_constants->robot()
                                        ->turret_constants()
                                        ->zeroing_constants()
                                        ->one_revolution_distance()),
            {.subsystem_params = {simulated_robot_constants->common()->turret(),
                                  simulated_robot_constants->robot()
                                      ->turret_constants()
                                      ->zeroing_constants()},
             .potentiometer_offset = simulated_robot_constants->robot()
                                         ->turret_constants()
                                         ->potentiometer_offset()},
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants->common()->turret()->range()),
            simulated_robot_constants->robot()
                ->turret_constants()
                ->zeroing_constants()
                ->measured_absolute_position(),
            dt_),
        extend_(
            new CappedTestPlant(extend::MakeExtendPlant()),
            PositionSensorSimulator(simulated_robot_constants->robot()
                                        ->extend_constants()
                                        ->zeroing_constants()
                                        ->one_revolution_distance()),
            {.subsystem_params = {simulated_robot_constants->common()->extend(),
                                  simulated_robot_constants->robot()
                                      ->extend_constants()
                                      ->zeroing_constants()},
             .potentiometer_offset = simulated_robot_constants->robot()
                                         ->extend_constants()
                                         ->potentiometer_offset()},
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants->common()->extend()->range()),
            simulated_robot_constants->robot()
                ->extend_constants()
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
    catapult_.InitializePosition(
        frc971::constants::Range::FromFlatbuffer(
            simulated_robot_constants->common()->catapult()->range())
            .middle());
    altitude_.InitializePosition(
        simulated_robot_constants->common()->altitude_loading_position());
    turret_.InitializePosition(
        simulated_robot_constants->common()->turret_loading_position());
    extend_.InitializePosition(
        frc971::constants::Range::FromFlatbuffer(
            simulated_robot_constants->common()->extend()->range())
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
            catapult_.Simulate(
                superstructure_output_fetcher_->catapult_voltage(),
                superstructure_status_fetcher_->shooter()->catapult());

            altitude_.Simulate(
                superstructure_output_fetcher_->altitude_voltage(),
                superstructure_status_fetcher_->shooter()->altitude());

            turret_.Simulate(
                superstructure_output_fetcher_->turret_voltage(),
                superstructure_status_fetcher_->shooter()->turret());

            extend_.Simulate(superstructure_output_fetcher_->extend_voltage(),
                             superstructure_status_fetcher_->extend());
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
    frc971::PotAndAbsolutePosition::Builder catapult_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();

    flatbuffers::Offset<frc971::PotAndAbsolutePosition> catapult_offset =
        catapult_.encoder()->GetSensorValues(&catapult_builder);

    frc971::PotAndAbsolutePosition::Builder altitude_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> altitude_offset =
        altitude_.encoder()->GetSensorValues(&altitude_builder);

    frc971::PotAndAbsolutePosition::Builder turret_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> turret_offset =
        turret_.encoder()->GetSensorValues(&turret_builder);

    frc971::PotAndAbsolutePosition::Builder extend_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> extend_offset =
        extend_.encoder()->GetSensorValues(&extend_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_extend_beambreak(extend_beambreak_);
    position_builder.add_catapult_beambreak(catapult_beambreak_);
    position_builder.add_intake_pivot(intake_pivot_offset);
    position_builder.add_catapult(catapult_offset);
    position_builder.add_altitude(altitude_offset);
    position_builder.add_turret(turret_offset);
    position_builder.add_climber(climber_offset);
    position_builder.add_extend(extend_offset);

    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

  void set_extend_beambreak(bool triggered) { extend_beambreak_ = triggered; }

  void set_catapult_beambreak(bool triggered) {
    catapult_beambreak_ = triggered;
  }

  AbsoluteEncoderSimulator *intake_pivot() { return &intake_pivot_; }
  PotAndAbsoluteEncoderSimulator *catapult() { return &catapult_; }
  PotAndAbsoluteEncoderSimulator *altitude() { return &altitude_; }
  PotAndAbsoluteEncoderSimulator *turret() { return &turret_; }
  PotAndAbsoluteEncoderSimulator *climber() { return &climber_; }

  PotAndAbsoluteEncoderSimulator *extend() { return &extend_; }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Sender<CANPosition> superstructure_can_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool extend_beambreak_;
  bool catapult_beambreak_;

  AbsoluteEncoderSimulator intake_pivot_;
  PotAndAbsoluteEncoderSimulator climber_;
  PotAndAbsoluteEncoderSimulator catapult_;
  PotAndAbsoluteEncoderSimulator altitude_;
  PotAndAbsoluteEncoderSimulator turret_;
  PotAndAbsoluteEncoderSimulator extend_;

  bool first_ = true;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2024/aos_config.json"),
            std::chrono::microseconds(5050)),
        simulated_constants_dummy_(SendSimulationConstants(
            event_loop_factory(), 7971, "y2024/constants/test_constants.json")),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        logger_pi_(aos::configuration::GetNode(configuration(), "logger")),
        superstructure_event_loop(MakeEventLoop("Superstructure", roborio_)),
        superstructure_(superstructure_event_loop.get()),
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
        superstructure_can_position_sender_(
            test_event_loop_->MakeSender<CANPosition>("/superstructure/rio")),
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

    EXPECT_FALSE(superstructure_status_fetcher_->collided());

    double set_point =
        superstructure_status_fetcher_->intake_pivot()->goal_position();

    if (superstructure_goal_fetcher_->intake_pivot() == IntakePivotGoal::DOWN) {
      set_point = simulated_robot_constants_->common()
                      ->intake_pivot_set_points()
                      ->extended();
    }

    EXPECT_NEAR(set_point,
                superstructure_status_fetcher_->intake_pivot()->position(),
                0.03);

    if (superstructure_goal_fetcher_->has_shooter_goal() &&
        superstructure_status_fetcher_->uncompleted_note_goal() !=
            NoteStatus::AMP &&
        superstructure_status_fetcher_->uncompleted_note_goal() !=
            NoteStatus::TRAP) {
      if (superstructure_goal_fetcher_->shooter_goal()->has_turret_position() &&
          !superstructure_goal_fetcher_->shooter_goal()->auto_aim()) {
        EXPECT_NEAR(
            superstructure_goal_fetcher_->shooter_goal()
                ->turret_position()
                ->unsafe_goal(),
            superstructure_status_fetcher_->shooter()->turret()->position(),
            0.001);
      }
    }

    if (superstructure_goal_fetcher_->has_shooter_goal()) {
      if (superstructure_goal_fetcher_->shooter_goal()
              ->has_altitude_position() &&
          !superstructure_goal_fetcher_->shooter_goal()->auto_aim() &&
          (superstructure_status_fetcher_->uncompleted_note_goal() !=
               NoteStatus::AMP &&
           superstructure_status_fetcher_->uncompleted_note_goal() !=
               NoteStatus::TRAP)) {
        EXPECT_NEAR(
            superstructure_goal_fetcher_->shooter_goal()
                ->altitude_position()
                ->unsafe_goal(),
            superstructure_status_fetcher_->shooter()->altitude()->position(),
            0.001);
        EXPECT_NEAR(superstructure_goal_fetcher_->shooter_goal()
                        ->altitude_position()
                        ->unsafe_goal(),
                    superstructure_plant_.altitude()->position(), 0.001);
      } else if (superstructure_status_fetcher_->uncompleted_note_goal() ==
                     NoteStatus::AMP ||
                 superstructure_status_fetcher_->uncompleted_note_goal() ==
                     NoteStatus::TRAP) {
        EXPECT_NEAR(
            simulated_robot_constants_->common()
                ->altitude_avoid_extend_collision_position(),
            superstructure_status_fetcher_->shooter()->altitude()->position(),
            0.001);
        EXPECT_NEAR(simulated_robot_constants_->common()
                        ->altitude_avoid_extend_collision_position(),
                    superstructure_plant_.altitude()->position(), 0.001);
      }
    }

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
      }

      if (superstructure_goal_fetcher_->climber_goal() == ClimberGoal::STOWED) {
        set_point = simulated_robot_constants_->common()
                        ->climber_set_points()
                        ->stowed();
      }
      EXPECT_NEAR(set_point,
                  superstructure_status_fetcher_->climber()->position(), 0.001);
    }

    if (superstructure_status_fetcher_->has_uncompleted_note_goal()) {
      double set_point = simulated_robot_constants_->common()
                             ->extend_set_points()
                             ->retracted();
      if (superstructure_status_fetcher_->uncompleted_note_goal() ==
          NoteStatus::TRAP) {
        set_point =
            simulated_robot_constants_->common()->extend_set_points()->trap();
      } else if (superstructure_status_fetcher_->uncompleted_note_goal() ==
                 NoteStatus::AMP) {
        set_point =
            simulated_robot_constants_->common()->extend_set_points()->amp();
      } else if (superstructure_status_fetcher_->uncompleted_note_goal() ==
                 NoteStatus::CATAPULT) {
        set_point = simulated_robot_constants_->common()
                        ->extend_set_points()
                        ->catapult();
      }

      EXPECT_NEAR(set_point,
                  superstructure_status_fetcher_->extend()->position(), 0.001);
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

  void WaitUntilNear(double turret_goal, double altitude_goal) {
    int i = 0;
    do {
      i++;
      RunFor(dt());
      superstructure_status_fetcher_.Fetch();
      // 10 Seconds

      ASSERT_LE(i, 10.0 / ::aos::time::DurationInSeconds(dt()));

      // Since there is a delay when sending running, make sure we have a
      // status before checking it.
    } while (superstructure_status_fetcher_.get() == nullptr ||
             std::abs(superstructure_plant_.altitude()->position() -
                      altitude_goal) > 1e-3 ||
             std::abs(superstructure_plant_.turret()->position() -
                      turret_goal) > 1e-3);
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
  ::aos::Sender<CANPosition> superstructure_can_position_sender_;
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

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(),
            simulated_robot_constants_->common()->turret_loading_position());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        altitude_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(),
            simulated_robot_constants_->common()->altitude_loading_position());

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_turret_position(turret_offset);
    shooter_goal_builder.add_altitude_position(altitude_offset);
    shooter_goal_builder.add_auto_aim(false);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_climber_goal(ClimberGoal::RETRACT);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_intake_pivot(IntakePivotGoal::UP);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);
  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::READY);
}

// Tests that loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(),
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants_->common()->turret()->range())
                .upper);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        altitude_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(),
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants_->common()->altitude()->range())
                .upper);

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_turret_position(turret_offset);
    shooter_goal_builder.add_altitude_position(altitude_offset);
    shooter_goal_builder.add_auto_aim(false);
    shooter_goal_builder.add_preloaded(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_climber_goal(ClimberGoal::FULL_EXTEND);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_catapult_beambreak(true);

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(15));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);
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
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), simulated_robot_constants_->common()
                                ->turret_avoid_extend_collision_position());

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        altitude_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), simulated_robot_constants_->common()
                                ->altitude_avoid_extend_collision_position());

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_turret_position(turret_offset);
    shooter_goal_builder.add_altitude_position(altitude_offset);
    shooter_goal_builder.add_auto_aim(false);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_climber_goal(ClimberGoal::FULL_EXTEND);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_note_goal(NoteGoal::AMP);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  superstructure_plant_.set_extend_beambreak(true);

  RunFor(chrono::seconds(30));
  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(),
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants_->common()->turret()->range())
                .lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 10));

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        altitude_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(),
            simulated_robot_constants_->common()
                ->altitude_avoid_extend_collision_position(),
            CreateProfileParameters(*builder.fbb(), 20.0, 10));

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_turret_position(turret_offset);
    shooter_goal_builder.add_altitude_position(altitude_offset);
    shooter_goal_builder.add_auto_aim(false);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_climber_goal(ClimberGoal::RETRACT);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_extend_beambreak(true);

  RunFor(chrono::seconds(10));
  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);
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

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.shooter().turret().state());

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.shooter().altitude().state());
  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.extend().state());
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
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

    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_intake_pivot(IntakePivotGoal::UP);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::SPIT);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(),
            simulated_robot_constants_->common()
                ->transfer_roller_voltages()
                ->transfer_out());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::INTAKING);

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(),
            simulated_robot_constants_->common()
                ->transfer_roller_voltages()
                ->transfer_in());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(500));

  // Make sure we're still intaking for 500 ms after we stop giving it an
  // intaking goal.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(200));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::INTAKING);
  EXPECT_EQ(superstructure_status_fetcher_->intake_roller(),
            IntakeRollerStatus::INTAKING);

  // Make sure we stop when loaded
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_extend_beambreak(true);

  RunFor(chrono::seconds(2));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);
}

// Tests the full range of activities we need to be doing from loading ->
// shooting
TEST_F(SuperstructureTest, LoadingToShooting) {
  SetEnabled(true);

  WaitUntilZeroed();

  constexpr double kTurretGoal = 2.0;
  constexpr double kAltitudeGoal = 0.55;

  set_alliance(aos::Alliance::kRed);
  SendDrivetrainStatus(0.0, {0.0, 5.0}, 0.0);

  // Auto aim, but don't fire.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_intake_pivot(IntakePivotGoal::UP);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_fire(false);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_catapult_beambreak(false);

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->turret()->position(),
              simulated_robot_constants_->common()->turret_loading_position(),
              0.01);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->altitude()->position(),
              simulated_robot_constants_->common()->altitude_loading_position(),
              0.01);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::READY);

  // Now, extend the intake.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_fire(false);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::INTAKING);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->turret()->position(),
              simulated_robot_constants_->common()->turret_loading_position(),
              0.01);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->altitude()->position(),
              simulated_robot_constants_->common()->altitude_loading_position(),
              0.01);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::READY);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_note_goal(NoteGoal::NONE);
    goal_builder.add_fire(false);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_extend_beambreak(false);

  RunFor(10 * dt());

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::INTAKING);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::READY);

  EXPECT_EQ(superstructure_status_fetcher_->extend_status(),
            ExtendStatus::RETRACTED);

  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::TRANSFERING_TO_EXTEND);

  EXPECT_LT(4.0, superstructure_output_fetcher_->transfer_roller_voltage());
  EXPECT_LT(4.0, superstructure_output_fetcher_->extend_roller_voltage());

  superstructure_plant_.set_extend_beambreak(true);

  RunFor(chrono::milliseconds(750));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::READY);

  EXPECT_EQ(superstructure_status_fetcher_->extend_status(),
            ExtendStatus::RETRACTED);
  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::IDLE);

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_output_fetcher_->extend_roller_voltage(), 0.0);

  // Verify we are in the loading position.
  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->turret()->position(),
              simulated_robot_constants_->common()->turret_loading_position(),
              0.01);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_note_goal(NoteGoal::CATAPULT);
    goal_builder.add_fire(false);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_extend_beambreak(true);

  RunFor(chrono::milliseconds(500));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADING_CATAPULT);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::READY);

  superstructure_plant_.set_catapult_beambreak(true);
  // Set retention roller to show that we are loaded.
  EXPECT_EQ(superstructure_status_fetcher_->extend_status(),
            ExtendStatus::CATAPULT);
  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::TRANSFERING_TO_CATAPULT);

  superstructure_plant_.set_extend_beambreak(false);

  RunFor(chrono::seconds(10));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::LOADED);

  EXPECT_EQ(superstructure_status_fetcher_->extend_status(),
            ExtendStatus::CATAPULT);
  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::IDLE);

  // Should now be loaded.
  superstructure_output_fetcher_.Fetch();
  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->altitude()->position(),
              simulated_robot_constants_->common()->altitude_loading_position(),
              0.01);

  // Fire.  Start by triggering a motion and then firing all in 1 go.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kTurretGoal);

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        altitude_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), kAltitudeGoal);

    auto catapult_builder =
        builder.MakeBuilder<frc971::control_loops::catapult::CatapultGoal>();
    catapult_builder.add_shot_velocity(15.0);

    const auto catapult_offset = catapult_builder.Finish();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(false);
    shooter_goal_builder.add_catapult_goal(catapult_offset);
    shooter_goal_builder.add_altitude_position(altitude_offset);
    shooter_goal_builder.add_turret_position(turret_offset);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_intake_pivot(IntakePivotGoal::UP);
    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_fire(true);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Wait until the bot finishes auto-aiming.
  WaitUntilNear(kTurretGoal, kAltitudeGoal);

  RunFor(dt());

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::FIRING);

  // Make sure it stays at firing for a bit.
  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::FIRING);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->turret()->position(),
              kTurretGoal, 0.001);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->altitude()->position(),
              kAltitudeGoal, 0.001);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(false);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter_goal(shooter_goal_offset);
    goal_builder.add_fire(false);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_catapult_beambreak(false);

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::READY);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->turret()->position(),
              simulated_robot_constants_->common()->turret_loading_position(),
              0.01);

  EXPECT_NEAR(superstructure_status_fetcher_->shooter()->altitude()->position(),
              simulated_robot_constants_->common()->altitude_loading_position(),
              0.01);
}

// Test that we are able to signal that the note was preloaded
TEST_F(SuperstructureTest, Preloaded) {
  // Put the bucket at the starting position.
  superstructure_plant_.catapult()->InitializePosition(
      simulated_robot_constants_->common()->catapult_return_position());

  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_preloaded(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter_goal(shooter_goal_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(200));

  superstructure_status_fetcher_.Fetch();
  ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);

  EXPECT_EQ(superstructure_status_fetcher_->shooter()->catapult_state(),
            CatapultState::LOADED);
}

// Tests that auto aim works properly for the shooter
TEST_F(SuperstructureTest, AutoAim) {
  superstructure_plant_.catapult()->InitializePosition(
      simulated_robot_constants_->common()->catapult_return_position());
  SetEnabled(true);
  WaitUntilZeroed();

  constexpr double kDistanceFromSpeaker = 4.0;

  const frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::ShotParams>
      interpolation_table =
          y2024::constants::Values::InterpolationTableFromFlatbuffer(
              simulated_robot_constants_->common()
                  ->shooter_interpolation_table());

  const double kRedSpeakerX = simulated_robot_constants_->common()
                                  ->shooter_targets()
                                  ->red_alliance()
                                  ->pos()
                                  ->data()
                                  ->Get(0);
  const double kRedSpeakerY = simulated_robot_constants_->common()
                                  ->shooter_targets()
                                  ->red_alliance()
                                  ->pos()
                                  ->data()
                                  ->Get(1);

  const double kBlueSpeakerX = simulated_robot_constants_->common()
                                   ->shooter_targets()
                                   ->blue_alliance()
                                   ->pos()
                                   ->data()
                                   ->Get(0);
  const double kBlueSpeakerY = simulated_robot_constants_->common()
                                   ->shooter_targets()
                                   ->blue_alliance()
                                   ->pos()
                                   ->data()
                                   ->Get(1);

  set_alliance(aos::Alliance::kRed);
  // Set the robot facing 90 degrees away from the speaker
  SendDrivetrainStatus(
      0.0, {(kRedSpeakerX - kDistanceFromSpeaker), kRedSpeakerY}, M_PI_2);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(true);
    shooter_goal_builder.add_preloaded(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter_goal(shooter_goal_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_catapult_beambreak(true);

  constants::Values::ShotParams shot_params;

  EXPECT_TRUE(
      interpolation_table.GetInRange(kDistanceFromSpeaker, &shot_params));

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);

  EXPECT_NEAR(
      -M_PI_2,
      superstructure_status_fetcher_->shooter()->aimer()->turret_position() -
          M_PI - 0.22,
      5e-4);
  EXPECT_NEAR(-M_PI_2,
              superstructure_status_fetcher_->shooter()->turret()->position() -
                  M_PI - 0.22,
              5e-4);

  EXPECT_EQ(
      kDistanceFromSpeaker,
      superstructure_status_fetcher_->shooter()->aimer()->target_distance());

  set_alliance(aos::Alliance::kBlue);
  // Set the robot facing 90 degrees away from the speaker
  SendDrivetrainStatus(
      0.0, {(kBlueSpeakerX + kDistanceFromSpeaker), kBlueSpeakerY}, M_PI_2);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<ShooterGoal>();

    shooter_goal_builder.add_auto_aim(true);

    flatbuffers::Offset<ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_shooter_goal(shooter_goal_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_catapult_beambreak(true);

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);

  EXPECT_NEAR(
      M_PI_2,
      superstructure_status_fetcher_->shooter()->aimer()->turret_position() +
          M_PI - 0.22,
      5e-4);
  EXPECT_NEAR(M_PI_2,
              superstructure_status_fetcher_->shooter()->turret()->position() +
                  M_PI - 0.22,
              5e-4);
  EXPECT_EQ(
      kDistanceFromSpeaker,
      superstructure_status_fetcher_->shooter()->aimer()->target_distance());
}

// Test entire sequence of loading, transfering, and scoring at amp position.
TEST_F(SuperstructureTest, ScoreInAmp) {
  SetEnabled(true);

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(),
            simulated_robot_constants_->common()
                ->transfer_roller_voltages()
                ->transfer_in());

  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::TRANSFERING_TO_EXTEND);

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::INTAKING);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::INTAKE);
    goal_builder.add_intake_pivot(IntakePivotGoal::DOWN);
    goal_builder.add_note_goal(NoteGoal::NONE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_extend_beambreak(true);

  RunFor(chrono::seconds(3));

  VerifyNearGoal();

  EXPECT_EQ(superstructure_output_fetcher_->transfer_roller_voltage(), 0.0);

  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::IDLE);

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::LOADED);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_intake_pivot(IntakePivotGoal::UP);
    goal_builder.add_note_goal(NoteGoal::AMP);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(10 * dt());

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_status_fetcher_->extend_status(),
            ExtendStatus::MOVING);

  RunFor(chrono::seconds(5));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_NEAR(superstructure_status_fetcher_->extend()->position(),
              simulated_robot_constants_->common()->extend_set_points()->amp(),
              0.01);
  EXPECT_EQ(superstructure_status_fetcher_->extend_status(), ExtendStatus::AMP);

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::READY);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_intake_pivot(IntakePivotGoal::UP);
    goal_builder.add_note_goal(NoteGoal::AMP);
    goal_builder.add_fire(true);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_extend_beambreak(true);

  RunFor(chrono::milliseconds(10));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());

  EXPECT_EQ(superstructure_status_fetcher_->state(),
            SuperstructureState::FIRING);

  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::SCORING_IN_AMP);
  EXPECT_LT(4.0, superstructure_output_fetcher_->extend_roller_voltage());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_intake_goal(IntakeGoal::NONE);
    goal_builder.add_intake_pivot(IntakePivotGoal::UP);
    goal_builder.add_note_goal(NoteGoal::AMP);
    goal_builder.add_fire(false);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.set_extend_beambreak(false);

  RunFor(chrono::milliseconds(100));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());

  EXPECT_EQ(superstructure_status_fetcher_->state(), SuperstructureState::IDLE);

  EXPECT_EQ(superstructure_status_fetcher_->extend_roller(),
            ExtendRollerStatus::IDLE);
  EXPECT_EQ(superstructure_output_fetcher_->extend_roller_voltage(), 0.0);
}

TEST_F(SuperstructureTest, Climbing) {
  SetEnabled(true);

  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_goal(ClimberGoal::STOWED);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_goal(ClimberGoal::FULL_EXTEND);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_climber_goal(ClimberGoal::RETRACT);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  // TODO(max): Fill this with the logic to move the altitude and turret on the
  // chain.
}
}  // namespace y2024::control_loops::superstructure::testing
