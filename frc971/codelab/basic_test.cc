#include "frc971/codelab/basic.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "aos/events/shm_event_loop.h"
#include "frc971/codelab/basic_generated.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace codelab {
namespace testing {

namespace chrono = ::std::chrono;
using aos::monotonic_clock;

// Class which simulates stuff and sends out queue messages with the
// position.
class BasicSimulation {
 public:
  BasicSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt)
      : event_loop_(event_loop),
        position_sender_(event_loop_->MakeSender<Position>("/codelab")),
        status_fetcher_(event_loop_->MakeFetcher<Status>("/codelab")),
        output_fetcher_(event_loop_->MakeFetcher<Output>("/codelab")) {
    event_loop_->AddPhasedLoop(
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

  // Sends a queue message with the position data.
  void SendPositionMessage() {
    auto builder = position_sender_.MakeBuilder();

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_limit_sensor(limit_sensor_);

    builder.Send(position_builder.Finish());
  }

  void VerifyResults(double voltage, bool status) {
    output_fetcher_.Fetch();
    status_fetcher_.Fetch();

    ASSERT_TRUE(output_fetcher_.get() != nullptr);
    ASSERT_TRUE(status_fetcher_.get() != nullptr);

    EXPECT_EQ(output_fetcher_->intake_voltage(), voltage);
    EXPECT_EQ(status_fetcher_->intake_complete(), status);
  }

  void set_limit_sensor(bool value) { limit_sensor_ = value; }

  // Simulates basic control loop for a single timestep.
  void Simulate() { EXPECT_TRUE(output_fetcher_.Fetch()); }

 private:
  ::aos::EventLoop *event_loop_;

  ::aos::Sender<Position> position_sender_;
  ::aos::Fetcher<Status> status_fetcher_;
  ::aos::Fetcher<Output> output_fetcher_;

  bool limit_sensor_ = false;

  bool first_ = true;
};

class BasicControlLoopTest : public ::aos::testing::ControlLoopTest {
 public:
  BasicControlLoopTest()
      : ::aos::testing::ControlLoopTest(
            "{\n"
            "  \"channels\": [ \n"
            "    {\n"
            "      \"name\": \"/aos\",\n"
            "      \"type\": \"aos.JoystickState\"\n"
            "    },\n"
            "    {\n"
            "      \"name\": \"/aos\",\n"
            "      \"type\": \"aos.RobotState\"\n"
            "    },\n"
            "    {\n"
            "      \"name\": \"/codelab\",\n"
            "      \"type\": \"frc971.codelab.Goal\"\n"
            "    },\n"
            "    {\n"
            "      \"name\": \"/codelab\",\n"
            "      \"type\": \"frc971.codelab.Output\"\n"
            "    },\n"
            "    {\n"
            "      \"name\": \"/codelab\",\n"
            "      \"type\": \"frc971.codelab.Status\"\n"
            "    },\n"
            "    {\n"
            "      \"name\": \"/codelab\",\n"
            "      \"type\": \"frc971.codelab.Position\"\n"
            "    }\n"
            "  ]\n"
            "}\n",
            chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop("test")),
        goal_sender_(test_event_loop_->MakeSender<Goal>("/codelab")),

        basic_event_loop_(MakeEventLoop("basic")),
        basic_(basic_event_loop_.get(), "/codelab"),

        basic_simulation_event_loop_(MakeEventLoop("simulation")),
        basic_simulation_(basic_simulation_event_loop_.get(), dt()) {
    set_team_id(control_loops::testing::kTeamNumber);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Sender<Goal> goal_sender_;

  ::std::unique_ptr<::aos::EventLoop> basic_event_loop_;
  Basic basic_;

  ::std::unique_ptr<::aos::EventLoop> basic_simulation_event_loop_;
  BasicSimulation basic_simulation_;
};

// Tests that when the motor has finished intaking it stops.
TEST_F(BasicControlLoopTest, IntakeLimitTransitionsToTrue) {
  {
    auto builder = goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(true);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  basic_simulation_.set_limit_sensor(false);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(12.0, false);

  {
    auto builder = goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(true);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  basic_simulation_.set_limit_sensor(true);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, true);
}

// Tests that the intake goes on if the sensor is not pressed
// and intake is requested.
TEST_F(BasicControlLoopTest, IntakeLimitNotSet) {
  {
    auto builder = goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(true);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  basic_simulation_.set_limit_sensor(false);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(12.0, false);
}

// Tests that the intake is off if no intake is requested,
// even if the limit sensor is off.
TEST_F(BasicControlLoopTest, NoIntakeLimitNotSet) {
  {
    auto builder = goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(false);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  basic_simulation_.set_limit_sensor(false);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, false);
}

// Tests that the intake is off even if the limit sensor
// is pressed and intake is requested.
TEST_F(BasicControlLoopTest, IntakeLimitSet) {
  {
    auto builder = goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(true);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  basic_simulation_.set_limit_sensor(true);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, true);
}

// Tests that the intake is off if no intake is requested,
TEST_F(BasicControlLoopTest, NoIntakeLimitSet) {
  {
    auto builder = goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(false);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  basic_simulation_.set_limit_sensor(true);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, false);
}

// Tests that the control loop handles things properly if no goal is set.
TEST_F(BasicControlLoopTest, NoGoalSet) {
  basic_simulation_.set_limit_sensor(true);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, false);
}

// Tests that the control loop handles things properly if disabled.
TEST_F(BasicControlLoopTest, Disabled) {
  basic_simulation_.set_limit_sensor(true);

  {
    auto builder = goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_intake(true);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  SetEnabled(false);
  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, false);
}

}  // namespace testing
}  // namespace codelab
}  // namespace frc971
