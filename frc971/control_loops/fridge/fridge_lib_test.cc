#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
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
      : left_arm_plant_(new StateFeedbackPlant<2, 1, 1>(MakeArmPlant())),
        right_arm_plant_(new StateFeedbackPlant<2, 1, 1>(MakeArmPlant())),
        left_elev_plant_(new StateFeedbackPlant<2, 1, 1>(MakeElevatorPlant())),
        right_elev_plant_(new StateFeedbackPlant<2, 1, 1>(MakeElevatorPlant())),
        fridge_queue_(".frc971.control_loops.fridge",
          0x78d8e372, ".frc971.control_loops.fridge.goal",
          ".frc971.control_loops.fridge.position",
          ".frc971.control_loops.fridge.output",
          ".frc971.control_loops.fridge.status") {
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::FridgeQueue::Position> position =
      fridge_queue_.position.MakeMessage();
    position.Send();
  }

  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(fridge_queue_.output.FetchLatest());
    left_arm_plant_->Update();
    right_arm_plant_->Update();
    left_elev_plant_->Update();
    right_elev_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> left_arm_plant_;
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> right_arm_plant_;
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> left_elev_plant_;
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> right_elev_plant_;

 private:
  FridgeQueue fridge_queue_;
};

class FridgeTest : public ::testing::Test {
 protected:
  FridgeTest() : fridge_queue_(".frc971.control_loops.fridge_queue",
                                   0x78d8e372,
                                   ".frc971.control_loops.fridge_queue.goal",
                                   ".frc971.control_loops.fridge_queue.position",
                                   ".frc971.control_loops.fridge_queue.output",
                                   ".frc971.control_loops.fridge_queue.status"),
                  fridge_(&fridge_queue_),
                  fridge_plant_() {
    // Flush the robot state queue so we can use clean shared memory for this
    // test.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
  }

  virtual ~FridgeTest() {
    ::aos::robot_state.Clear();
  }

  // Update the robot state. Without this, the Iteration of the control loop
  // will stop all the motors and the shooter won't go anywhere.
  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled)
                                        .autonomous(false)
                                        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

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

  // Bring up and down Core.
  ::aos::common::testing::GlobalCoreInstance my_core;

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
  SendDSPacket(true);
  fridge_plant_.SendPositionMessage();
  fridge_.Iterate();
  fridge_plant_.Simulate();
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
