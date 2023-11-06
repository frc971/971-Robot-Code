#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/subsystem_simulator.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2023_bot3/constants/simulated_constants_sender.h"
#include "y2023_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2023_bot3/control_loops/superstructure/superstructure.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2023_bot3 {
namespace control_loops {
namespace superstructure {
namespace testing {
namespace {}  // namespace
namespace chrono = std::chrono;

using ::aos::monotonic_clock;
using ::frc971::CreateProfileParameters;
using ::frc971::control_loops::CappedTestPlant;
using ::frc971::control_loops::
    CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using ::frc971::control_loops::PositionSensorSimulator;
using ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using DrivetrainStatus = ::frc971::control_loops::drivetrain::Status;
using PotAndAbsoluteEncoderSimulator =
    frc971::control_loops::SubsystemSimulator<
        frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
        Superstructure::PotAndAbsoluteEncoderSubsystem::State,
        constants::Values::PotAndAbsEncoderConstants>;

// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation(::aos::EventLoop *event_loop,
                           std::shared_ptr<const constants::Values> values,
                           chrono::nanoseconds dt)
      : event_loop_(event_loop),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")),
        pivot_joint_(new CappedTestPlant(pivot_joint::MakePivotJointPlant()),
                     PositionSensorSimulator(
                         values->pivot_joint.subsystem_params.zeroing_constants
                             .one_revolution_distance),
                     values->pivot_joint, constants::Values::kPivotJointRange(),
                     values->pivot_joint.subsystem_params.zeroing_constants
                         .measured_absolute_position,
                     dt) {
    (void)values;
    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
            EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

            pivot_joint_.Simulate(
                superstructure_output_fetcher_->pivot_joint_voltage(),
                superstructure_status_fetcher_->pivot_joint());
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

    frc971::PotAndAbsolutePosition::Builder pivot_joint_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> pivot_joint_offset =
        pivot_joint_.encoder()->GetSensorValues(&pivot_joint_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();
    position_builder.add_end_effector_cube_beam_break(
        end_effector_cube_beam_break_);
    position_builder.add_pivot_joint_position(pivot_joint_offset);

    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

  void set_end_effector_cube_beam_break(bool triggered) {
    end_effector_cube_beam_break_ = triggered;
  }

  PotAndAbsoluteEncoderSimulator *pivot_joint() { return &pivot_joint_; }

 private:
  ::aos::EventLoop *event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  bool end_effector_cube_beam_break_ = false;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  PotAndAbsoluteEncoderSimulator pivot_joint_;

  bool first_ = true;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2023_bot3/aos_config.json"),
            std::chrono::microseconds(5050)),
        values_(std::make_shared<constants::Values>(constants::MakeValues())),
        simulated_constants_dummy_(SendSimulationConstants(
            event_loop_factory(), 7971,
            "y2023_bot3/constants/test_constants.json")),
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
      logger_->StartLoggingOnRun(FLAGS_output_folder);
    }
  }

  void VerifyNearGoal() {
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();

    ASSERT_TRUE(superstructure_goal_fetcher_.get() != nullptr) << ": No goal";
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr)
        << ": No status";

    if (superstructure_goal_fetcher_->has_pivot_goal()) {
      double pivot_goal = 0.0;

      switch (superstructure_goal_fetcher_->pivot_goal()) {
        case PivotGoal::NEUTRAL:
          pivot_goal = 0;
          break;

        case PivotGoal::PICKUP_FRONT:
          pivot_goal = 0.25;
          break;

        case PivotGoal::PICKUP_BACK:
          pivot_goal = 0.30;
          break;

        case PivotGoal::SCORE_LOW_FRONT:
          pivot_goal = 0.35;
          break;

        case PivotGoal::SCORE_LOW_BACK:
          pivot_goal = 0.40;
          break;

        case PivotGoal::SCORE_MID_FRONT:
          pivot_goal = 0.45;
          break;

        case PivotGoal::SCORE_MID_BACK:
          pivot_goal = 0.5;
          break;
      }

      EXPECT_NEAR(pivot_goal,
                  superstructure_status_fetcher_->pivot_joint()->position(),
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

  std::shared_ptr<const constants::Values> values_;
  const bool simulated_constants_dummy_;

  const aos::Node *const roborio_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2023_bot3::control_loops::superstructure::Superstructure superstructure_;
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

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(20));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // TODO(Milo): Make this a sane time
  RunFor(chrono::seconds(20));
  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
}

TEST_F(SuperstructureTest, PivotGoal) {
  SetEnabled(true);
  WaitUntilZeroed();

  PivotGoal pivot_goal = PivotGoal::PICKUP_FRONT;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(pivot_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 30);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  VerifyNearGoal();

  pivot_goal = PivotGoal::PICKUP_BACK;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(pivot_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 30);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  VerifyNearGoal();

  pivot_goal = PivotGoal::SCORE_LOW_FRONT;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(pivot_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 30);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  VerifyNearGoal();

  pivot_goal = PivotGoal::SCORE_LOW_BACK;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(pivot_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 30);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  VerifyNearGoal();

  pivot_goal = PivotGoal::SCORE_MID_FRONT;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(pivot_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 30);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  VerifyNearGoal();

  pivot_goal = PivotGoal::SCORE_MID_BACK;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(pivot_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 30);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  VerifyNearGoal();

  pivot_goal = PivotGoal::NEUTRAL;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(pivot_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 30);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  VerifyNearGoal();
}

TEST_F(SuperstructureTest, EndEffectorGoal) {
  SetEnabled(true);
  WaitUntilZeroed();

  double spit_voltage = EndEffector::kRollerCubeSpitVoltage();
  double suck_voltage = EndEffector::kRollerCubeSuckVoltage();

  RollerGoal roller_goal = RollerGoal::INTAKE_CUBE;

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_goal(roller_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_end_effector_cube_beam_break(false);

  // This makes sure that we intake as normal when
  // requesting intake.
  RunFor(constants::Values::kExtraIntakingTime());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), suck_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::INTAKING);

  superstructure_plant_.set_end_effector_cube_beam_break(true);

  // Checking that after the beambreak is set once intaking that the
  // state changes to LOADED.
  RunFor(dt());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::LOADED);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_goal(RollerGoal::IDLE);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_end_effector_cube_beam_break(false);

  //  Checking that it's going back to intaking because we lost the
  //  beambreak sensor.
  RunFor(dt() * 2);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), suck_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::INTAKING);

  // Checking that we go back to idle after beambreak is lost and we
  // set our goal to idle.
  RunFor(dt() * 2 + constants::Values::kExtraIntakingTime());
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::IDLE);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_goal(roller_goal);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  // Going through intake -> loaded -> spitting
  // Making sure that it's intaking normally.
  RunFor(constants::Values::kExtraIntakingTime());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), suck_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::INTAKING);

  superstructure_plant_.set_end_effector_cube_beam_break(true);

  // Checking that it's loaded once beambreak is sensing something.
  RunFor(dt());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::LOADED);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_roller_goal(RollerGoal::SPIT);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_end_effector_cube_beam_break(true);
  // Checking that it stays spitting until 2 seconds after the
  // beambreak is lost.
  RunFor(dt() * 10);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), spit_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::SPITTING);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_roller_goal(RollerGoal::IDLE);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  // Checking that it goes to idle after it's given time to stop spitting.
  RunFor(dt() * 3);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::IDLE);
}

// Test that we are able to signal that the cube was preloaded
TEST_F(SuperstructureTest, Preloaded) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_preloaded_with_cube(true);
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(dt());

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::LOADED);
}

// Tests that the end effector does nothing when the goal is to remain
// still.
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_roller_goal(RollerGoal::IDLE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));
  VerifyNearGoal();

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
}
// Tests that loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_roller_goal(RollerGoal::IDLE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  // Give it a lot of time to get there.
  RunFor(chrono::seconds(15));

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023_bot3
