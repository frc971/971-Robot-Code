#include "y2016/control_loops/superstructure/superstructure.h"

#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/controls/control_loop_test.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"

using ::aos::time::Time;

namespace y2016 {
namespace control_loops {
namespace testing {

// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation()
      : superstructure_queue_(".y2016.control_loops.superstructure", 0x0,
                               ".y2016.control_loops.superstructure.goal",
                               ".y2016.control_loops.superstructure.status",
                               ".y2016.control_loops.superstructure.output",
                               ".y2016.control_loops.superstructure.status") {
  }

  // Simulates superstructure for a single timestep.
  void Simulate() { EXPECT_TRUE(superstructure_queue_.output.FetchLatest()); }

 private:
  SuperstructureQueue superstructure_queue_;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : superstructure_queue_(".y2016.control_loops.superstructure", 0x0,
                               ".y2016.control_loops.superstructure.goal",
                               ".y2016.control_loops.superstructure.status",
                               ".y2016.control_loops.superstructure.output",
                               ".y2016.control_loops.superstructure.status"),
        superstructure_(&superstructure_queue_),
        superstructure_plant_() {}

  void VerifyNearGoal() {
    // TODO(phil): Implement a check here.
  }

  // Runs one iteration of the whole simulation and checks that separation
  // remains reasonable.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);
    TickTime();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const Time &run_for, bool enabled = true) {
    const auto start_time = Time::Now();
    while (Time::Now() < start_time + run_for) {
      RunIteration(enabled);
    }
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  SuperstructureQueue superstructure_queue_;

  // Create a control loop and simulation.
  Superstructure superstructure_;
  SuperstructureSimulation superstructure_plant_;
};

// Tests that the superstructure does nothing when the goal is zero.
TEST_F(SuperstructureTest, DoesNothing) {
  // TODO(phil): Send a goal of some sort.
  RunIteration();
  VerifyNearGoal();
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
