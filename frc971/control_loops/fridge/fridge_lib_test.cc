#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/controls/control_loop_test.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/fridge/fridge.h"
#include "frc971/constants.h"

using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

// Class which simulates the fridge and sends out queue messages with the
// position.
class FridgeSimulation {
 public:
  // Constructs a simulation.
  FridgeSimulation()
      : arm_plant_(new StateFeedbackPlant<4, 2, 2>(MakeArmPlant())),
        elev_plant_(new StateFeedbackPlant<4, 2, 2>(MakeElevatorPlant())),
        fridge_queue_(".frc971.control_loops.fridge_queue", 0xe4e05855,
                      ".frc971.control_loops.fridge_queue.goal",
                      ".frc971.control_loops.fridge_queue.position",
                      ".frc971.control_loops.fridge_queue.output",
                      ".frc971.control_loops.fridge_queue.status") {}

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::FridgeQueue::Position> position =
        fridge_queue_.position.MakeMessage();
    position.Send();
  }

  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(fridge_queue_.output.FetchLatest());
    arm_plant_->Update();
    elev_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> arm_plant_;
  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> elev_plant_;

 private:
  FridgeQueue fridge_queue_;
};

class FridgeTest : public ::aos::testing::ControlLoopTest {
 protected:
  FridgeTest()
      : fridge_queue_(".frc971.control_loops.fridge_queue", 0xe4e05855,
                      ".frc971.control_loops.fridge_queue.goal",
                      ".frc971.control_loops.fridge_queue.position",
                      ".frc971.control_loops.fridge_queue.output",
                      ".frc971.control_loops.fridge_queue.status"),
        fridge_(&fridge_queue_),
        fridge_plant_() {}

  void VerifyNearGoal() {
    fridge_queue_.goal.FetchLatest();
    fridge_queue_.status.FetchLatest();
    EXPECT_NEAR(fridge_queue_.goal->angle,
                fridge_queue_.status->angle,
                10.0);
    EXPECT_NEAR(fridge_queue_.goal->height,
                fridge_queue_.status->height,
                10.0);
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  FridgeQueue fridge_queue_;

  // Create a control loop and simulation.
  Fridge fridge_;
  FridgeSimulation fridge_plant_;
};

// Tests that the loop does nothing when the goal is zero.
TEST_F(FridgeTest, DoesNothing) {
  fridge_queue_.goal.MakeWithBuilder().angle(0.0).height(0.0).Send();
  SendMessages(true);
  fridge_plant_.SendPositionMessage();
  fridge_.Iterate();
  fridge_plant_.Simulate();
  TickTime();
  VerifyNearGoal();
  fridge_queue_.output.FetchLatest();
  EXPECT_EQ(fridge_queue_.output->left_arm, 0.0);
  EXPECT_EQ(fridge_queue_.output->right_arm, 0.0);
  EXPECT_EQ(fridge_queue_.output->left_elevator, 0.0);
  EXPECT_EQ(fridge_queue_.output->right_elevator, 0.0);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
