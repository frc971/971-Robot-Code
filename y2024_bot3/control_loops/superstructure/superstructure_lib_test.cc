#include <chrono>
#include <memory>

#include "absl/flags/flag.h"
#include "gtest/gtest.h"

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/subsystem_simulator.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "y2024_bot3/constants/simulated_constants_sender.h"
#include "y2024_bot3/control_loops/superstructure/arm/arm_plant.h"
#include "y2024_bot3/control_loops/superstructure/superstructure.h"

ABSL_FLAG(std::string, output_folder, "",
          "If set, logs all channels to the provided logfile.");

namespace y2024_bot3::control_loops::superstructure::testing {

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
        arm_(new CappedTestPlant(arm::MakeArmPlant()),
             PositionSensorSimulator(simulated_robot_constants->robot()
                                         ->arm_constants()
                                         ->zeroing_constants()
                                         ->one_revolution_distance()),
             {.subsystem_params = {simulated_robot_constants->common()->arm(),
                                   simulated_robot_constants->robot()
                                       ->arm_constants()
                                       ->zeroing_constants()},
              .potentiometer_offset = simulated_robot_constants->robot()
                                          ->arm_constants()
                                          ->arm_potentiometer_offset()},
             frc971::constants::Range::FromFlatbuffer(
                 simulated_robot_constants->common()->arm()->range()),
             simulated_robot_constants->robot()
                 ->arm_constants()
                 ->zeroing_constants()
                 ->measured_absolute_position(),
             dt_)

  {
    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
            EXPECT_TRUE(superstructure_status_fetcher_.Fetch());
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

    frc971::PotAndAbsolutePosition::Builder arm_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> arm_offset =
        arm_.encoder()->GetSensorValues(&arm_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_arm(arm_offset);

    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  PotAndAbsoluteEncoderSimulator arm_;

  bool first_ = true;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2024_bot3/aos_config.json"),
            std::chrono::microseconds(5000)),
        simulated_constants_dummy_(SendSimulationConstants(
            event_loop_factory(), 9971,
            "y2024_bot3/constants/test_constants.json")),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        logger_pi_(aos::configuration::GetNode(configuration(), "logger")),
        superstructure_event_loop(MakeEventLoop("Superstructure", roborio_)),
        superstructure_(superstructure_event_loop.get()),
        test_event_loop_(MakeEventLoop("test", roborio_)),
        constants_fetcher_(test_event_loop_.get()),
        simulated_robot_constants_(&constants_fetcher_.constants()),
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
        superstructure_plant_event_loop_(MakeEventLoop("plant", roborio_)),
        superstructure_plant_(superstructure_plant_event_loop_.get(),
                              simulated_robot_constants_, dt()) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);

    SetEnabled(true);

    if (!absl::GetFlag(FLAGS_output_folder).empty()) {
      unlink(absl::GetFlag(FLAGS_output_folder).c_str());
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(absl::GetFlag(FLAGS_output_folder));
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

    double set_point;
    auto arm_positions =
        simulated_robot_constants_->robot()->arm_constants()->arm_positions();

    switch (superstructure_goal_fetcher_->arm_position()) {
      case PivotGoal::IDLE:
        set_point = arm_positions->idle();
        break;
      case PivotGoal::INTAKE:
        set_point = arm_positions->intake();
        break;
      case PivotGoal::AMP:
        set_point = arm_positions->amp();
        break;
      default:
        LOG(FATAL) << "PivotGoal is not IDLE, INTAKE, or AMP";
    }

    EXPECT_NEAR(
        set_point,
        superstructure_status_fetcher_->arm()->unprofiled_goal_position(),
        0.03);
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

  void WaitUntilNear() {}

  const bool simulated_constants_dummy_;

  const aos::Node *const roborio_;
  const aos::Node *const logger_pi_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2024_bot3::control_loops::superstructure::Superstructure superstructure_;
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
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
}

TEST_F(SuperstructureTest, ArmPivotMovement) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_position(PivotGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(20));
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->arm_status(), ArmStatus::INTAKE);

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_position(PivotGoal::AMP);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->arm_status(), ArmStatus::AMP);

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_position(PivotGoal::IDLE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(superstructure_status_fetcher_->arm_status(), ArmStatus::IDLE);

  VerifyNearGoal();
}

}  // namespace y2024_bot3::control_loops::superstructure::testing
