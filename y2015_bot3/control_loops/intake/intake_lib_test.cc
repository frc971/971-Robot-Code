#include "y2015_bot3/control_loops/intake/intake.h"

#include <math.h>
#include <unistd.h>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/controls/control_loop_test.h"
#include "y2015_bot3/control_loops/intake/intake.q.h"

using ::aos::time::Time;

namespace y2015_bot3 {
namespace control_loops {
namespace testing {

// Class which simulates the elevator and sends out queue messages with the
// position.
class IntakeSimulation {
 public:
  // Constructs a simulation.
  IntakeSimulation()
      : queue_(".y2015_bot3.control_loops.intake_queue", 0x627ceeeb,
               ".y2015_bot3.control_loops.intake_queue.goal",
               ".y2015_bot3.control_loops.intake_queue.position",
               ".y2015_bot3.control_loops.intake_queue.output",
               ".y2015_bot3.control_loops.intake_queue.status") {}

  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(queue_.output.FetchLatest());
  }

 private:
  IntakeQueue queue_;
};

class IntakeTest : public ::aos::testing::ControlLoopTest {
 protected:
  IntakeTest()
      : queue_(".y2015_bot3.control_loops.intake_queue", 0x627ceeeb,
               ".y2015_bot3.control_loops.intake_queue.goal",
               ".y2015_bot3.control_loops.intake_queue.position",
               ".y2015_bot3.control_loops.intake_queue.output",
               ".y2015_bot3.control_loops.intake_queue.status"),
        intake_(&queue_),
        plant_() {
    set_team_id(971);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    queue_.position.MakeMessage().Send();

    intake_.Iterate();

    TickTime();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(double run_for, bool enabled = true) {
    const auto start_time = Time::Now();
    while (Time::Now() < start_time + Time::InSeconds(run_for)) {
      RunIteration(enabled);
    }
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to. Otherwise, we will have a pointed to shared memory that
  // is no longer valid.
  IntakeQueue queue_;

  // Create a control loop.
  Intake intake_;
  IntakeSimulation plant_;
};

// Tests that the loop does nothing when the goal is zero.
TEST_F(IntakeTest, DoesNothing) {
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .movement(0)
                  .Send());

  // Run for a bit.
  RunForTime(1.0);

  ASSERT_TRUE(queue_.output.FetchLatest());
  EXPECT_EQ(queue_.output->intake, 0.0);
}

// Tests that the intake sucks with a positive goal.
TEST_F(IntakeTest, SuckingGoal) {
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .movement(1)
                  .Send());

  // Run for a bit.
  RunForTime(1.0);

  ASSERT_TRUE(queue_.output.FetchLatest());
  EXPECT_GT(queue_.output->intake, 0.0);
}

// Tests that the intake spits with a negative goal.
TEST_F(IntakeTest, SpittingGoal) {
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .movement(-1)
                  .Send());

  // Run for a bit.
  RunForTime(1.0);

  ASSERT_TRUE(queue_.output.FetchLatest());
  EXPECT_LT(queue_.output->intake, 0.0);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace y2015_bot3
