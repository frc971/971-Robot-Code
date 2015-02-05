#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/controls/control_loop_test.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw.h"
#include "frc971/control_loops/position_sensor_sim.h"
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
        pot_and_encoder_(0.0,
                         constants::GetValues().claw.wrist.lower_hard_limit,
                         constants::GetValues().claw_index_diff,
                         0.3),
        claw_queue_(".frc971.control_loops.claw_queue",
          0x9d7452fb, ".frc971.control_loops.claw_queue.goal",
          ".frc971.control_loops.claw_queue.position",
          ".frc971.control_loops.claw_queue.output",
          ".frc971.control_loops.claw_queue.status") {
  }

  // Do specific initialization for the sensors.
  void SetSensors(double start_value, double pot_noise_stddev) {
    pot_and_encoder_.OverrideParams(start_value, pot_noise_stddev);
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::ClawQueue::Position> position =
      claw_queue_.position.MakeMessage();
    pot_and_encoder_.GetSensorValues(&position->joint);
    position.Send();
  }

  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(claw_queue_.output.FetchLatest());

    // Feed voltages into physics simulation.
    claw_plant_->mutable_U() << claw_queue_.output->voltage;
    claw_plant_->Update();

    const double wrist_angle = claw_plant_->Y(0, 0);

    // TODO(danielp): Sanity checks.

    pot_and_encoder_.MoveTo(wrist_angle);
  }

 private:
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> claw_plant_;
  PositionSensorSimulator pot_and_encoder_;

  ClawQueue claw_queue_;
};

class ClawTest : public ::aos::testing::ControlLoopTest {
 protected:
  ClawTest()
      : claw_queue_(".frc971.control_loops.claw_queue", 0x9d7452fb,
                    ".frc971.control_loops.claw_queue.goal",
                    ".frc971.control_loops.claw_queue.position",
                    ".frc971.control_loops.claw_queue.output",
                    ".frc971.control_loops.claw_queue.status"),
        claw_(&claw_queue_),
        claw_plant_() {}

  void VerifyNearGoal() {
    claw_queue_.goal.FetchLatest();
    claw_queue_.status.FetchLatest();
    EXPECT_NEAR(claw_queue_.goal->angle,
                claw_queue_.status->angle,
                10.0);
  }

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
  SendMessages(true);
  claw_plant_.SendPositionMessage();
  claw_.Iterate();
  claw_plant_.Simulate();
  TickTime();
  VerifyNearGoal();
  claw_queue_.output.FetchLatest();
  EXPECT_EQ(claw_queue_.output->voltage, 0.0);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
