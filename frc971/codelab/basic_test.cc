#include "frc971/codelab/basic.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "aos/events/shm-event-loop.h"
#include "aos/queue.h"
#include "frc971/codelab/basic.q.h"
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
        position_sender_(event_loop_->MakeSender<BasicQueue::Position>(
            ".frc971.codelab.basic_queue.position")),
        status_fetcher_(event_loop_->MakeFetcher<BasicQueue::Status>(
            ".frc971.codelab.basic_queue.status")),
        output_fetcher_(event_loop_->MakeFetcher<BasicQueue::Output>(
            ".frc971.codelab.basic_queue.output")) {
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
    auto position = position_sender_.MakeMessage();

    position->limit_sensor = limit_sensor_;

    position.Send();
  }

  void VerifyResults(double voltage, bool status) {
    output_fetcher_.Fetch();
    status_fetcher_.Fetch();

    ASSERT_TRUE(output_fetcher_.get() != nullptr);
    ASSERT_TRUE(status_fetcher_.get() != nullptr);

    EXPECT_EQ(output_fetcher_->intake_voltage, voltage);
    EXPECT_EQ(status_fetcher_->intake_complete, status);
  }

  void set_limit_sensor(bool value) { limit_sensor_ = value; }

  // Simulates basic control loop for a single timestep.
  void Simulate() { EXPECT_TRUE(output_fetcher_.Fetch()); }

 private:
  ::aos::EventLoop *event_loop_;

  ::aos::Sender<BasicQueue::Position> position_sender_;
  ::aos::Fetcher<BasicQueue::Status> status_fetcher_;
  ::aos::Fetcher<BasicQueue::Output> output_fetcher_;

  bool limit_sensor_ = false;

  bool first_ = true;
};

class BasicControlLoopTest : public ::aos::testing::ControlLoopTest {
 public:
  BasicControlLoopTest()
      : ::aos::testing::ControlLoopTest(chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop()),
        goal_sender_(test_event_loop_->MakeSender<BasicQueue::Goal>(
            ".frc971.codelab.basic_queue.goal")),

        basic_event_loop_(MakeEventLoop()),
        basic_(basic_event_loop_.get(), ".frc971.codelab.basic_queue"),

        basic_simulation_event_loop_(MakeEventLoop()),
        basic_simulation_(basic_simulation_event_loop_.get(), dt()) {
    set_team_id(control_loops::testing::kTeamNumber);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Sender<BasicQueue::Goal> goal_sender_;

  ::std::unique_ptr<::aos::EventLoop> basic_event_loop_;
  Basic basic_;

  ::std::unique_ptr<::aos::EventLoop> basic_simulation_event_loop_;
  BasicSimulation basic_simulation_;
};

// Tests that when the motor has finished intaking it stops.
TEST_F(BasicControlLoopTest, IntakeLimitTransitionsToTrue) {
  {
    auto message = goal_sender_.MakeMessage();
    message->intake = true;
    ASSERT_TRUE(message.Send());
  }

  basic_simulation_.set_limit_sensor(false);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(12.0, false);

  {
    auto message = goal_sender_.MakeMessage();
    message->intake = true;
    ASSERT_TRUE(message.Send());
  }
  basic_simulation_.set_limit_sensor(true);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, true);
}

// Tests that the intake goes on if the sensor is not pressed
// and intake is requested.
TEST_F(BasicControlLoopTest, IntakeLimitNotSet) {
  {
    auto message = goal_sender_.MakeMessage();
    message->intake = true;
    ASSERT_TRUE(message.Send());
  }
  basic_simulation_.set_limit_sensor(false);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(12.0, false);
}

// Tests that the intake is off if no intake is requested,
// even if the limit sensor is off.
TEST_F(BasicControlLoopTest, NoIntakeLimitNotSet) {
  {
    auto message = goal_sender_.MakeMessage();
    message->intake = false;
    ASSERT_TRUE(message.Send());
  }
  basic_simulation_.set_limit_sensor(false);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, false);
}

// Tests that the intake is off even if the limit sensor
// is pressed and intake is requested.
TEST_F(BasicControlLoopTest, IntakeLimitSet) {
  {
    auto message = goal_sender_.MakeMessage();
    message->intake = true;
    ASSERT_TRUE(message.Send());
  }
  basic_simulation_.set_limit_sensor(true);

  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, true);
}

// Tests that the intake is off if no intake is requested,
TEST_F(BasicControlLoopTest, NoIntakeLimitSet) {
  {
    auto message = goal_sender_.MakeMessage();
    message->intake = false;
    ASSERT_TRUE(message.Send());
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
    auto message = goal_sender_.MakeMessage();
    message->intake = true;
    ASSERT_TRUE(message.Send());
  }

  SetEnabled(false);
  RunFor(dt() * 2);

  basic_simulation_.VerifyResults(0.0, false);
}

}  // namespace testing
}  // namespace codelab
}  // namespace frc971
