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
#include "y2025/constants.h"
#include "y2025/constants/constants_generated.h"
#include "y2025/constants/simulated_constants_sender.h"
#include "y2025/control_loops/superstructure/elevator/elevator_plant.h"
#include "y2025/control_loops/superstructure/pivot/pivot_plant.h"
#include "y2025/control_loops/superstructure/superstructure.h"
#include "y2025/control_loops/superstructure/superstructure_can_position_generated.h"
#include "y2025/control_loops/superstructure/superstructure_position_generated.h"
#include "y2025/control_loops/superstructure/wrist/wrist_plant.h"

ABSL_FLAG(std::string, output_folder, "",
          "If set, logs all channels to the provided logfile.");

namespace y2025::control_loops::superstructure::testing {

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
using PotAndAbsoluteEncoderSimulator =
    frc971::control_loops::SubsystemSimulator<
        frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
        PotAndAbsoluteEncoderSubsystem::State,
        constants::Values::PotAndAbsEncoderConstants>;

using AbsoluteEncoderSimulator = ::frc971::control_loops::SubsystemSimulator<
    ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus,
    Superstructure::AbsoluteEncoderSubsystem::State,
    constants::Values::AbsEncoderConstants>;

typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;

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
        elevator_(new CappedTestPlant(elevator::MakeElevatorPlant()),
                  PositionSensorSimulator(simulated_robot_constants->robot()
                                              ->elevator_constants()
                                              ->zeroing_constants()
                                              ->one_revolution_distance()),
                  {.subsystem_params =
                       {simulated_robot_constants->common()->elevator(),
                        simulated_robot_constants->robot()
                            ->elevator_constants()
                            ->zeroing_constants()},
                   .potentiometer_offset = simulated_robot_constants->robot()
                                               ->elevator_constants()
                                               ->potentiometer_offset()},
                  frc971::constants::Range::FromFlatbuffer(
                      simulated_robot_constants->common()->elevator()->range()),
                  simulated_robot_constants->robot()
                      ->elevator_constants()
                      ->zeroing_constants()
                      ->measured_absolute_position(),
                  dt_),
        pivot_(
            new CappedTestPlant(pivot::MakepivotPlant()),
            PositionSensorSimulator(simulated_robot_constants->robot()
                                        ->pivot_constants()
                                        ->zeroing_constants()
                                        ->one_revolution_distance()),
            {.subsystem_params = {simulated_robot_constants->common()->pivot(),
                                  simulated_robot_constants->robot()
                                      ->pivot_constants()
                                      ->zeroing_constants()},
             .potentiometer_offset = simulated_robot_constants->robot()
                                         ->pivot_constants()
                                         ->potentiometer_offset()},
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants->common()->pivot()->range()),
            simulated_robot_constants->robot()
                ->pivot_constants()
                ->zeroing_constants()
                ->measured_absolute_position(),
            dt_),
        wrist_(
            new CappedTestPlant(wrist::MakeWristPlant()),
            PositionSensorSimulator(simulated_robot_constants->robot()
                                        ->wrist_constants()
                                        ->zeroing_constants()
                                        ->one_revolution_distance()),
            {.subsystem_params = {simulated_robot_constants->common()->wrist(),
                                  simulated_robot_constants->robot()
                                      ->wrist_constants()
                                      ->zeroing_constants()}},
            frc971::constants::Range::FromFlatbuffer(
                simulated_robot_constants->common()->wrist()->range()),
            simulated_robot_constants->robot()
                ->wrist_constants()
                ->zeroing_constants()
                ->measured_absolute_position(),
            dt_) {
    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
            EXPECT_TRUE(superstructure_status_fetcher_.Fetch());
            elevator_.Simulate(
                superstructure_output_fetcher_->elevator_voltage(),
                superstructure_status_fetcher_->elevator());
            pivot_.Simulate(superstructure_output_fetcher_->pivot_voltage(),
                            superstructure_status_fetcher_->pivot());
            wrist_.Simulate(superstructure_output_fetcher_->wrist_voltage(),
                            superstructure_status_fetcher_->wrist());
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
  }

  AbsoluteEncoderSimulator *wrist() { return &wrist_; }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::PotAndAbsolutePosition::Builder elevator_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> elevator_offset =
        elevator_.encoder()->GetSensorValues(&elevator_builder);

    frc971::PotAndAbsolutePosition::Builder pivot_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> pivot_offset =
        pivot_.encoder()->GetSensorValues(&pivot_builder);

    frc971::AbsolutePosition::Builder wrist_builder =
        builder.MakeBuilder<frc971::AbsolutePosition>();
    flatbuffers::Offset<frc971::AbsolutePosition> wrist_offset =
        wrist_.encoder()->GetSensorValues(&wrist_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();
    position_builder.add_pivot(pivot_offset);
    position_builder.add_elevator(elevator_offset);
    position_builder.add_wrist(wrist_offset);

    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Sender<CANPosition> superstructure_can_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  PotAndAbsoluteEncoderSimulator elevator_;
  PotAndAbsoluteEncoderSimulator pivot_;
  AbsoluteEncoderSimulator wrist_;

  bool first_ = true;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2025/aos_config.json"),
            std::chrono::microseconds(5000)),
        simulated_constants_dummy_(SendSimulationConstants(
            event_loop_factory(), 971, "y2025/constants/test_constants.json")),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
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
        superstructure_can_position_sender_(
            test_event_loop_->MakeSender<CANPosition>("/superstructure/rio")),
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
    constexpr double kEpsPivot = 0.001;
    constexpr double kEpsElevator = 0.001;
    constexpr double kEpsWrist = 0.001;
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();
    superstructure_output_fetcher_.Fetch();
    superstructure_position_fetcher_.Fetch();

    double expected_end_effector_voltage =
        simulated_robot_constants_->common()->end_effector_idling_voltage();
    EndEffectorStatus expected_end_effector_status = EndEffectorStatus::NEUTRAL;
    switch (superstructure_goal_fetcher_->end_effector_goal()) {
      case EndEffectorGoal::NEUTRAL:
        break;
      case EndEffectorGoal::INTAKE:
        if (!superstructure_.GetIntakeComplete()) {
          expected_end_effector_status = EndEffectorStatus::INTAKING;
          expected_end_effector_voltage = simulated_robot_constants_->common()
                                              ->end_effector_voltages()
                                              ->intake();
        }
        break;
      case EndEffectorGoal::SPIT:
        expected_end_effector_status = EndEffectorStatus::SPITTING;
        expected_end_effector_voltage = simulated_robot_constants_->common()
                                            ->end_effector_voltages()
                                            ->spit();
        break;
    }

    EXPECT_EQ(superstructure_output_fetcher_->end_effector_voltage(),
              expected_end_effector_voltage);
    EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
              expected_end_effector_status);

    double elevator_expected_position =
        simulated_robot_constants_->common()->elevator_set_points()->neutral();

    switch (superstructure_goal_fetcher_->elevator_goal()) {
      case ElevatorGoal::NEUTRAL:
        break;
      case ElevatorGoal::INTAKE:
        elevator_expected_position = simulated_robot_constants_->common()
                                         ->elevator_set_points()
                                         ->intake();
        break;
      case ElevatorGoal::SCORE_L1:
        elevator_expected_position = simulated_robot_constants_->common()
                                         ->elevator_set_points()
                                         ->score_l1();
        break;
      case ElevatorGoal::SCORE_L2:
        elevator_expected_position = simulated_robot_constants_->common()
                                         ->elevator_set_points()
                                         ->score_l2();
        break;
      case ElevatorGoal::SCORE_L3:
        elevator_expected_position = simulated_robot_constants_->common()
                                         ->elevator_set_points()
                                         ->score_l3();
        break;
      case ElevatorGoal::SCORE_L4:
        elevator_expected_position = simulated_robot_constants_->common()
                                         ->elevator_set_points()
                                         ->score_l4();
        break;
    }

    EXPECT_NEAR(elevator_expected_position,
                superstructure_status_fetcher_->elevator()->position(),
                kEpsElevator);

    auto pivot_positions =
        simulated_robot_constants_->common()->pivot_set_points();
    double pivot_set_point = pivot_positions->neutral();

    switch (superstructure_goal_fetcher_->pivot_goal()) {
      case PivotGoal::NEUTRAL:
        break;
      case PivotGoal::SCORE_L1:
        pivot_set_point = pivot_positions->score_l1();
        break;
      case PivotGoal::SCORE_L2:
        pivot_set_point = pivot_positions->score_l2();
        break;
      case PivotGoal::SCORE_L3:
        pivot_set_point = pivot_positions->score_l3();
        break;
      case PivotGoal::SCORE_L4:
        pivot_set_point = pivot_positions->score_l4();
        break;
      case PivotGoal::INTAKE:
        pivot_set_point = pivot_positions->intake();
        break;
    }
    EXPECT_NEAR(pivot_set_point,
                superstructure_status_fetcher_->pivot()->position(), kEpsPivot);

    const y2025::WristSetPoints *wrist_positions =
        simulated_robot_constants_->common()->wrist_set_points();
    double wrist_set_point = wrist_positions->intake();

    switch (superstructure_goal_fetcher_->wrist_goal()) {
      case WristGoal::INTAKE:
        wrist_set_point = wrist_positions->intake();
        break;
      case WristGoal::SCORE_L1:
        wrist_set_point = wrist_positions->score_l1();
        break;
      case WristGoal::SCORE_L2:
        wrist_set_point = wrist_positions->score_l2();
        break;
      case WristGoal::SCORE_L3:
        wrist_set_point = wrist_positions->score_l3();
        break;
      case WristGoal::SCORE_L4:
        wrist_set_point = wrist_positions->score_l4();
        break;
      case WristGoal::NEUTRAL:
        wrist_set_point = wrist_positions->neutral();
        break;
    }
    EXPECT_NEAR(wrist_set_point,
                superstructure_status_fetcher_->wrist()->position(), kEpsWrist);

    double climber_expected_voltage = 0.0;

    switch (superstructure_goal_fetcher_->climber_goal()) {
      case ClimberGoal::NEUTRAL:
        break;
      case ClimberGoal::CLIMB:
        climber_expected_voltage =
            simulated_robot_constants_->common()->climber_voltage()->climb();
        break;
      case ClimberGoal::RETRACT:
        climber_expected_voltage =
            simulated_robot_constants_->common()->climber_voltage()->retract();
        break;
    }

    ASSERT_TRUE(climber_expected_voltage ==
                superstructure_output_fetcher_->climber_voltage());

    ASSERT_FALSE(superstructure_status_fetcher_->estopped());

    ASSERT_TRUE(superstructure_goal_fetcher_.get() != nullptr) << ": No goal";
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr)
        << ": No status";
    ASSERT_TRUE(superstructure_output_fetcher_.get() != nullptr)
        << ": No output";
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

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2025::control_loops::superstructure::Superstructure superstructure_;
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

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
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

  RunFor(chrono::seconds(1));

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

TEST_F(SuperstructureTest, ClimberPositionTest) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_climber_goal(ClimberGoal::NEUTRAL);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_climber_goal(ClimberGoal::CLIMB);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_climber_goal(ClimberGoal::RETRACT);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();
}

TEST_F(SuperstructureTest, ElevatorPositionTest) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_elevator_goal(ElevatorGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_elevator_goal(ElevatorGoal::SCORE_L1);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_elevator_goal(ElevatorGoal::SCORE_L2);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_elevator_goal(ElevatorGoal::SCORE_L3);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_elevator_goal(ElevatorGoal::SCORE_L4);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(3));

  VerifyNearGoal();
}

TEST_F(SuperstructureTest, WristPositionTest) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_wrist_goal(WristGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(3));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_wrist_goal(WristGoal::SCORE_L2);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(3));

  VerifyNearGoal();
}

TEST_F(SuperstructureTest, PivotPositionTest) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(PivotGoal::NEUTRAL);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(3));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(PivotGoal::SCORE_L2);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(3));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(PivotGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(3));

  VerifyNearGoal();
}

TEST_F(SuperstructureTest, EndEffectorTest) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_end_effector_goal(EndEffectorGoal::NEUTRAL);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_end_effector_goal(EndEffectorGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_end_effector_goal(EndEffectorGoal::SPIT);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_end_effector_goal(EndEffectorGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_can_position_sender_.MakeBuilder();
    frc971::control_loops::CANTalonFX::Builder talon_builder =
        builder.MakeBuilder<frc971::control_loops::CANTalonFX>();
    talon_builder.add_torque_current(1000);
    auto talon_torque_offset = talon_builder.Finish();
    CANPosition::Builder can_builder = builder.MakeBuilder<CANPosition>();
    can_builder.add_end_effector(talon_torque_offset);

    ASSERT_EQ(builder.Send(can_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();
}

TEST_F(SuperstructureTest, PivotAndElevatorAndWristPositionTest) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(PivotGoal::NEUTRAL);
    goal_builder.add_elevator_goal(ElevatorGoal::INTAKE);
    goal_builder.add_wrist_goal(WristGoal::INTAKE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_pivot_goal(PivotGoal::SCORE_L1);
    goal_builder.add_elevator_goal(ElevatorGoal::SCORE_L1);
    goal_builder.add_wrist_goal(WristGoal::SCORE_L1);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  VerifyNearGoal();
}

}  // namespace y2025::control_loops::superstructure::testing
