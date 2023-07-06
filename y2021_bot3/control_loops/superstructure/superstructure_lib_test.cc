#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2021_bot3/control_loops/superstructure/superstructure.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2021_bot3 {
namespace control_loops {
namespace superstructure {
namespace testing {

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2021_bot3/aos_config.json"),
            std::chrono::microseconds(5050)),
        superstructure_event_loop(MakeEventLoop("Superstructure")),
        superstructure_(superstructure_event_loop.get()),
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
        superstructure_position_sender_(
            test_event_loop_->MakeSender<Position>("/superstructure")) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);
    SetEnabled(true);

    phased_loop_handle_ = test_event_loop_->AddPhasedLoop(
        [this](int) { SendPositionMessage(); }, dt());

    if (!FLAGS_output_folder.empty()) {
      unlink(FLAGS_output_folder.c_str());
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_folder);
    }
  }

  void VerifyResults(double intake_voltage, double outtake_voltage,
                     double climber_voltage, double intake_speed,
                     double outtake_speed, double climber_speed) {
    superstructure_output_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_output_fetcher_.get() != nullptr);
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);
    EXPECT_EQ(superstructure_output_fetcher_->intake_volts(), intake_voltage);
    EXPECT_EQ(superstructure_output_fetcher_->outtake_volts(), outtake_voltage);
    EXPECT_EQ(superstructure_output_fetcher_->climber_volts(), climber_voltage);
    EXPECT_EQ(superstructure_status_fetcher_->intake_speed(), intake_speed);
    EXPECT_EQ(superstructure_status_fetcher_->outtake_speed(), outtake_speed);
    EXPECT_EQ(superstructure_status_fetcher_->climber_speed(), climber_speed);
    EXPECT_EQ(superstructure_status_fetcher_->estopped(), false);
    EXPECT_EQ(superstructure_status_fetcher_->zeroed(), true);
  }

  void SendPositionMessage() {
    auto builder = superstructure_position_sender_.MakeBuilder();
    Position::Builder position_builder = builder.MakeBuilder<Position>();
    builder.CheckOk(builder.Send(position_builder.Finish()));
  }

  // Because the third robot is single node, the roborio node is nullptr
  const aos::Node *const roborio_ = nullptr;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2021_bot3::control_loops::superstructure::Superstructure superstructure_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;
  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;
  ::aos::Sender<Position> superstructure_position_sender_;
  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

// Tests running the intake and outtake separately
TEST_F(SuperstructureTest, RunIntakeOrOuttake) {
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_speed(10.0);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    SendPositionMessage();
    RunFor(dt() * 2);
    VerifyResults(10.0, 0.0, 0.0, 10.0, 0.0, 0.0);
  }

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_outtake_speed(10.0);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    VerifyResults(0.0, 10.0, 0.0, 0.0, 10.0, 0.0);
  }
}

TEST_F(SuperstructureTest, RunClimber) {
  auto builder = superstructure_goal_sender_.MakeBuilder();
  Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
  goal_builder.add_climber_speed(4.0);
  builder.CheckOk(builder.Send(goal_builder.Finish()));
  RunFor(dt() * 2);
  VerifyResults(0.0, 0.0, 4.0, 0.0, 0.0, 4.0);
}

// Tests running both the intake and the outtake simultaneously
TEST_F(SuperstructureTest, RunIntakeAndOuttake) {
  auto builder = superstructure_goal_sender_.MakeBuilder();
  Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
  goal_builder.add_intake_speed(10.0);
  goal_builder.add_outtake_speed(5.0);
  builder.CheckOk(builder.Send(goal_builder.Finish()));
  RunFor(dt() * 2);
  VerifyResults(10.0, 5.0, 0.0, 10.0, 5.0, 0.0);
}

// Tests for an invalid voltage (over 12 or under -12) to check that it defaults
// to 12 or -12
TEST_F(SuperstructureTest, InvalidVoltage) {
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_speed(20.0);
    goal_builder.add_outtake_speed(15.0);
    goal_builder.add_climber_speed(18.0);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    VerifyResults(12.0, 12.0, 12.0, 20.0, 15.0, 18.0);
  }

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake_speed(-20.0);
    goal_builder.add_outtake_speed(-15.0);
    goal_builder.add_climber_speed(-18.0);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
    RunFor(dt() * 2);
    VerifyResults(-12.0, -12.0, -12.0, -20.0, -15.0, -18.0);
  }
}

// Tests that there is no output when the goal is null
TEST_F(SuperstructureTest, GoalIsNull) {
  auto builder = superstructure_goal_sender_.MakeBuilder();
  Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
  builder.CheckOk(builder.Send(goal_builder.Finish()));
  RunFor(dt() * 2);
  VerifyResults(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Tests that the robot behaves properly when disabled
TEST_F(SuperstructureTest, Disabled) {
  auto builder = superstructure_goal_sender_.MakeBuilder();
  Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
  goal_builder.add_intake_speed(6.0);
  goal_builder.add_outtake_speed(5.0);
  goal_builder.add_climber_speed(4.0);
  builder.CheckOk(builder.Send(goal_builder.Finish()));
  SetEnabled(false);
  RunFor(dt() * 2);
  VerifyResults(0.0, 0.0, 0.0, 6.0, 5.0, 4.0);
}

TEST_F(SuperstructureTest, PlotterTest) {
  double speed = 10.0;
  test_event_loop_->AddPhasedLoop(
      [&](int) {
        auto builder = superstructure_goal_sender_.MakeBuilder();
        Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
        goal_builder.add_intake_speed(speed);
        goal_builder.add_outtake_speed(speed);
        goal_builder.add_climber_speed(speed);
        builder.CheckOk(builder.Send(goal_builder.Finish()));
        speed += .001;
        if (speed >= 12) {
          speed = -12;
        }
      },
      frc971::controls::kLoopFrequency);
  RunFor(std::chrono::seconds(10));
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2021_bot3
