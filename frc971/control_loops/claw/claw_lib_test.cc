#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw.h"
#include "frc971/constants.h"

using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

// Class which simulates the claw and sends out queue messages with the
// position.
class ClawSimulation {
 public:
  // Constructs a claw simulation.
  ClawSimulation()
      : claw_plant_(new StateFeedbackPlant<2, 1, 1>(MakeClawPlant())),
        claw_queue_(".frc971.control_loops.claw_queue",
          0x78d8e372, ".frc971.control_loops.claw_queue.goal",
          ".frc971.control_loops.claw_queue.position",
          ".frc971.control_loops.claw_queue.output",
          ".frc971.control_loops.claw_queue.status") {
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::ClawQueue::Position> position =
      claw_queue_.position.MakeMessage();
    position.Send();
  }

  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(claw_queue_.output.FetchLatest());
    claw_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> claw_plant_;

 private:
  ClawQueue claw_queue_;
};

class ClawTest : public ::testing::Test {
 protected:
  ClawTest() : claw_queue_(".frc971.control_loops.claw_queue",
                                   0x78d8e372,
                                   ".frc971.control_loops.claw_queue.goal",
                                   ".frc971.control_loops.claw_queue.position",
                                   ".frc971.control_loops.claw_queue.output",
                                   ".frc971.control_loops.claw_queue.status"),
                  claw_(&claw_queue_),
                  claw_plant_() {
    // Flush the robot state queue so we can use clean shared memory for this
    // test.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
  }

  virtual ~ClawTest() {
    ::aos::robot_state.Clear();
  }

  // Update the robot state. Without this, the Iteration of the control loop
  // will stop all the motors and this won't go anywhere.
  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled)
                                        .autonomous(false)
                                        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

  void VerifyNearGoal() {
    claw_queue_.goal.FetchLatest();
    claw_queue_.status.FetchLatest();
    EXPECT_NEAR(claw_queue_.goal->angle,
                claw_queue_.status->angle,
                10.0);
  }

  // Bring up and down Core.
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  ClawQueue claw_queue_;

  // Create a control loop and simulation.
  Claw claw_;
  ClawSimulation claw_plant_;
};

// Tests that the loop does nothing when the goal is zero.
TEST_F(ClawTest, DoesNothing) {
  claw_queue_.goal.MakeWithBuilder().angle(0.0).Send();
  SendDSPacket(true);
  claw_plant_.SendPositionMessage();
  claw_.Iterate();
  claw_plant_.Simulate();
  VerifyNearGoal();
  claw_queue_.output.FetchLatest();
  EXPECT_EQ(claw_queue_.output->voltage, 0.0);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
