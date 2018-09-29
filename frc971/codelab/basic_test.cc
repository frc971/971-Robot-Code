#include "frc971/codelab/basic.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "aos/queue.h"
#include "frc971/codelab/basic.q.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace codelab {
namespace testing {

// Class which simulates stuff and sends out queue messages with the
// position.
class BasicSimulation {
 public:
  BasicSimulation()
      : basic_queue_(".frc971.codelab.basic_queue", 0x78d8e372,
                     ".frc971.codelab.basic_queue.goal",
                     ".frc971.codelab.basic_queue.position",
                     ".frc971.codelab.basic_queue.output",
                     ".frc971.codelab.basic_queue.status") {}

  // Sends a queue message with the position data.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<BasicQueue::Position> position =
        basic_queue_.position.MakeMessage();

    position->limit_sensor = limit_sensor_;

    position.Send();
  }

  void VerifyResults(double voltage, bool status) {
    basic_queue_.output.FetchLatest();
    basic_queue_.status.FetchLatest();

    ASSERT_TRUE(basic_queue_.output.get() != nullptr);
    ASSERT_TRUE(basic_queue_.status.get() != nullptr);

    EXPECT_EQ(basic_queue_.output->intake_voltage, voltage);
    EXPECT_EQ(basic_queue_.status->intake_complete, status);
  }

  void set_limit_sensor(bool value) { limit_sensor_ = value; }

  // Simulates basic control loop for a single timestep.
  void Simulate() { EXPECT_TRUE(basic_queue_.output.FetchLatest()); }

 private:
  BasicQueue basic_queue_;
  bool limit_sensor_ = false;
};

class BasicControlLoopTest : public ::aos::testing::ControlLoopTest {
 public:
  BasicControlLoopTest()
      : basic_queue_(".frc971.codelab.basic_queue", 0x78d8e372,
                     ".frc971.codelab.basic_queue.goal",
                     ".frc971.codelab.basic_queue.position",
                     ".frc971.codelab.basic_queue.output",
                     ".frc971.codelab.basic_queue.status"),
        basic_loop_(&basic_queue_) {
    set_team_id(control_loops::testing::kTeamNumber);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    basic_simulation_.SendPositionMessage();
    basic_loop_.Iterate();
    basic_simulation_.Simulate();

    TickTime();
  }

  BasicQueue basic_queue_;
  Basic basic_loop_;
  BasicSimulation basic_simulation_;
};

// Tests that when the motor has finished intaking it stops.
TEST_F(BasicControlLoopTest, IntakeLimitTransitionsToTrue) {
  ASSERT_TRUE(basic_queue_.goal.MakeWithBuilder().intake(true).Send());
  basic_simulation_.set_limit_sensor(false);

  RunIteration();

  basic_simulation_.VerifyResults(12.0, false);

  ASSERT_TRUE(basic_queue_.goal.MakeWithBuilder().intake(true).Send());
  basic_simulation_.set_limit_sensor(true);

  RunIteration();

  basic_simulation_.VerifyResults(0.0, true);
}

// Tests that the intake goes on if the sensor is not pressed
// and intake is requested.
TEST_F(BasicControlLoopTest, IntakeLimitNotSet) {
  ASSERT_TRUE(basic_queue_.goal.MakeWithBuilder().intake(true).Send());
  basic_simulation_.set_limit_sensor(false);

  RunIteration();

  basic_simulation_.VerifyResults(12.0, false);
}

// Tests that the intake is off if no intake is requested,
// even if the limit sensor is off.
TEST_F(BasicControlLoopTest, NoIntakeLimitNotSet) {
  ASSERT_TRUE(basic_queue_.goal.MakeWithBuilder().intake(false).Send());
  basic_simulation_.set_limit_sensor(false);

  RunIteration();

  basic_simulation_.VerifyResults(0.0, false);
}

// Tests that the intake is off even if the limit sensor
// is pressed and intake is requested.
TEST_F(BasicControlLoopTest, IntakeLimitSet) {
  ASSERT_TRUE(basic_queue_.goal.MakeWithBuilder().intake(true).Send());
  basic_simulation_.set_limit_sensor(true);

  RunIteration();

  basic_simulation_.VerifyResults(0.0, true);
}

// Tests that the intake is off if no intake is requested,
TEST_F(BasicControlLoopTest, NoIntakeLimitSet) {
  ASSERT_TRUE(basic_queue_.goal.MakeWithBuilder().intake(false).Send());
  basic_simulation_.set_limit_sensor(true);

  RunIteration();

  basic_simulation_.VerifyResults(0.0, false);
}

// Tests that the control loop handles things properly if no goal is set.
TEST_F(BasicControlLoopTest, NoGoalSet) {
  basic_simulation_.set_limit_sensor(true);

  RunIteration();

  basic_simulation_.VerifyResults(0.0, false);
}

// Tests that the control loop handles things properly if disabled.
TEST_F(BasicControlLoopTest, Disabled) {
  basic_simulation_.set_limit_sensor(true);

  ASSERT_TRUE(basic_queue_.goal.MakeWithBuilder().intake(true).Send());

  RunIteration(false);

  basic_simulation_.VerifyResults(0.0, false);
}

}  // namespace testing
}  // namespace codelab
}  // namespace frc971
