#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "bot3/control_loops/shooter/shooter_motor.q.h"
#include "bot3/control_loops/shooter/shooter.h"
#include "frc971/constants.h"

using ::aos::time::Time;

namespace bot3 {
namespace control_loops {
namespace testing {

// Class which simulates the shooter and sends out queue messages with the
// position.
class ShooterMotorSimulation {
 public:
  // Constructs a shooter simulation. I'm not sure how the construction of
  // the queue (my_shooter_loop_) actually works (same format as wrist).
  ShooterMotorSimulation()
      : shooter_plant_(new StateFeedbackPlant<2, 1, 1>(MakeShooterPlant())),
        my_shooter_loop_(".bot3.control_loops.shooter",
          0x78d8e372, ".bot3.control_loops.shooter.goal",
          ".bot3.control_loops.shooter.position",
          ".bot3.control_loops.shooter.output",
          ".bot3.control_loops.shooter.status") {
  }

  // Sends a queue message with the position of the shooter.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::ShooterLoop::Position> position =
      my_shooter_loop_.position.MakeMessage();
    position->position = shooter_plant_->Y(0, 0);
    position.Send();
  }

  // Simulates shooter for a single timestep.
  void Simulate() {
    EXPECT_TRUE(my_shooter_loop_.output.FetchLatest());
    shooter_plant_->U << my_shooter_loop_.output->voltage;
    shooter_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> shooter_plant_;

 private:
  ShooterLoop my_shooter_loop_;
};

class ShooterTest : public ::testing::Test {
 protected:
  ShooterTest() : my_shooter_loop_(".bot3.control_loops.shooter",
                                   0x78d8e372,
                                   ".bot3.control_loops.shooter.goal",
                                   ".bot3.control_loops.shooter.position",
                                   ".bot3.control_loops.shooter.output",
                                   ".bot3.control_loops.shooter.status"),
                  shooter_motor_(&my_shooter_loop_),
                  shooter_motor_plant_() {
    // Flush the robot state queue so we can use clean shared memory for this
    // test.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
  }

  virtual ~ShooterTest() {
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
    my_shooter_loop_.goal.FetchLatest();
    my_shooter_loop_.status.FetchLatest();
    EXPECT_NEAR(my_shooter_loop_.goal->velocity,
                my_shooter_loop_.status->average_velocity,
                10.0);
  }

  // Bring up and down Core.
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  ShooterLoop my_shooter_loop_;

  // Create a control loop and simulation.
  ShooterMotor shooter_motor_;
  ShooterMotorSimulation shooter_motor_plant_;
};

// Tests that the shooter does nothing when the goal is zero.
TEST_F(ShooterTest, DoesNothing) {
  my_shooter_loop_.goal.MakeWithBuilder().velocity(0.0).Send();
  SendDSPacket(true);
  shooter_motor_plant_.SendPositionMessage();
  shooter_motor_.Iterate();
  shooter_motor_plant_.Simulate();
  VerifyNearGoal();
  my_shooter_loop_.output.FetchLatest();
  EXPECT_EQ(my_shooter_loop_.output->voltage, 0.0);
}

// Tests that the shooter spins up to speed and that it then spins down
// without applying any power.
TEST_F(ShooterTest, SpinUpAndDown) {
  my_shooter_loop_.goal.MakeWithBuilder().velocity(450.0).Send();
  bool is_done = false;
  while (!is_done) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
    EXPECT_TRUE(my_shooter_loop_.status.FetchLatest());
    is_done = my_shooter_loop_.status->ready;
  }
  VerifyNearGoal();

  my_shooter_loop_.goal.MakeWithBuilder().velocity(0.0).Send();
  for (int i = 0; i < 100; ++i) {
    shooter_motor_plant_.SendPositionMessage();
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
    EXPECT_TRUE(my_shooter_loop_.output.FetchLatest());
    EXPECT_EQ(0.0, my_shooter_loop_.output->voltage);
  }
}

// Tests that the shooter can spin up nicely while missing position packets.
TEST_F(ShooterTest, MissingPositionMessages) {
  my_shooter_loop_.goal.MakeWithBuilder().velocity(200.0).Send();
  for (int i = 0; i < 100; ++i) {
    if (i % 7) {
      shooter_motor_plant_.SendPositionMessage();
    }
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(true);
  }

  VerifyNearGoal();
}

// Tests that the shooter can spin up nicely while disabled for a while.
TEST_F(ShooterTest, Disabled) {
  my_shooter_loop_.goal.MakeWithBuilder().velocity(200.0).Send();
  for (int i = 0; i < 100; ++i) {
    if (i % 7) {
      shooter_motor_plant_.SendPositionMessage();
    }
    shooter_motor_.Iterate();
    shooter_motor_plant_.Simulate();
    SendDSPacket(i > 50);
  }

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace control_loops
}  // namespace bot3
