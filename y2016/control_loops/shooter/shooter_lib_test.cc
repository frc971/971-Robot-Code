#include "y2016/control_loops/shooter/shooter.h"

#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/controls/control_loop_test.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/control_loops/shooter/shooter.h"

using ::aos::time::Time;
using ::frc971::control_loops::testing::kTeamNumber;

namespace y2016 {
namespace control_loops {
namespace testing {

// Class which simulates the shooter and sends out queue messages with the
// position.
class ShooterSimulation {
 public:
  // Constructs a shooter simulation.
  ShooterSimulation()
      : shooter_plant_left_(new StateFeedbackPlant<2, 1, 1>(
            ::y2016::control_loops::shooter::MakeShooterPlant())),
        shooter_plant_right_(new StateFeedbackPlant<2, 1, 1>(
            ::y2016::control_loops::shooter::MakeShooterPlant())),
        shooter_queue_(".y2016.control_loops.shooter", 0x78d8e372,
                       ".y2016.control_loops.shooter.goal",
                       ".y2016.control_loops.shooter.position",
                       ".y2016.control_loops.shooter.output",
                       ".y2016.control_loops.shooter.status") {}

  // Sends a queue message with the position of the shooter.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::ShooterQueue::Position> position =
        shooter_queue_.position.MakeMessage();

    position->theta_left = shooter_plant_left_->Y(0, 0);
    position->theta_right = shooter_plant_right_->Y(0, 0);
    position.Send();
  }

  // Simulates shooter for a single timestep.
  void Simulate() {
    EXPECT_TRUE(shooter_queue_.output.FetchLatest());

    shooter_plant_left_->mutable_U(0, 0) = shooter_queue_.output->voltage_left;
    shooter_plant_right_->mutable_U(0, 0) =
        shooter_queue_.output->voltage_right;

    shooter_plant_left_->Update();
    shooter_plant_right_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> shooter_plant_left_,
      shooter_plant_right_;

 private:
  ShooterQueue shooter_queue_;
};

class ShooterTest : public ::aos::testing::ControlLoopTest {
 protected:
  ShooterTest()
      : shooter_queue_(".y2016.control_loops.shooter", 0x78d8e372,
                       ".y2016.control_loops.shooter.goal",
                       ".y2016.control_loops.shooter.position",
                       ".y2016.control_loops.shooter.output",
                       ".y2016.control_loops.shooter.status"),
        shooter_(&shooter_queue_),
        shooter_plant_() {
    set_team_id(kTeamNumber);
  }

  void VerifyNearGoal() {
    shooter_queue_.goal.FetchLatest();
    shooter_queue_.status.FetchLatest();

    EXPECT_TRUE(shooter_queue_.goal.get() != nullptr);
    EXPECT_TRUE(shooter_queue_.status.get() != nullptr);

    EXPECT_NEAR(shooter_queue_.goal->angular_velocity_left,
                shooter_queue_.status->avg_angular_velocity_left, 10.0);
    EXPECT_NEAR(shooter_queue_.goal->angular_velocity_right,
                shooter_queue_.status->avg_angular_velocity_right, 10.0);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    shooter_plant_.SendPositionMessage();
    shooter_.Iterate();
    shooter_plant_.Simulate();

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
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ShooterQueue shooter_queue_;

  // Create a control loop and simulation.
  Shooter shooter_;
  ShooterSimulation shooter_plant_;
};

// Tests that the shooter does nothing when the goal is zero.
TEST_F(ShooterTest, DoesNothing) {
  ASSERT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity_left(0.0)
                  .angular_velocity_right(0.0)
                  .Send());
  RunIteration();

  VerifyNearGoal();

  EXPECT_TRUE(shooter_queue_.output.FetchLatest());
  EXPECT_EQ(shooter_queue_.output->voltage_left, 0.0);
  EXPECT_EQ(shooter_queue_.output->voltage_right, 0.0);
}

// Tests that the shooter spins up to speed and that it then spins down
// without applying any power.
TEST_F(ShooterTest, SpinUpAndDown) {
  // Spin up.
  EXPECT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity_left(450.0)
                  .angular_velocity_right(450.0)
                  .Send());
  RunForTime(Time::InSeconds(10));
  VerifyNearGoal();
  EXPECT_TRUE(shooter_queue_.status->ready_both);
  EXPECT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity_left(0.0)
                  .angular_velocity_right(0.0)
                  .Send());

  // Make sure we don't apply voltage on spin-down.
  RunIteration();
  EXPECT_TRUE(shooter_queue_.output.FetchLatest());
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_left);
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_right);

  // Continue to stop.
  RunForTime(Time::InSeconds(5));
  EXPECT_TRUE(shooter_queue_.output.FetchLatest());
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_left);
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_right);
}

// Tests that the shooter doesn't say it is ready if one side isn't up to speed.
// According to our tuning, we may overshoot the goal on one shooter and
// mistakenly say that we are ready. This test should look at both extremes.
TEST_F(ShooterTest, SideLagTest) {
  // Spin up.
  EXPECT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity_left(20.0)
                  .angular_velocity_right(450.0)
                  .Send());
  RunForTime(Time::InSeconds(0.1));

  shooter_queue_.goal.FetchLatest();
  shooter_queue_.status.FetchLatest();

  EXPECT_TRUE(shooter_queue_.goal.get() != nullptr);
  EXPECT_TRUE(shooter_queue_.status.get() != nullptr);

  // Left should be up to speed, right shouldn't.
  EXPECT_TRUE(shooter_queue_.status->ready_left);
  EXPECT_FALSE(shooter_queue_.status->ready_right);
  EXPECT_FALSE(shooter_queue_.status->ready_both);

  RunForTime(Time::InSeconds(5));

  shooter_queue_.goal.FetchLatest();
  shooter_queue_.status.FetchLatest();

  EXPECT_TRUE(shooter_queue_.goal.get() != nullptr);
  EXPECT_TRUE(shooter_queue_.status.get() != nullptr);

  // Both should be up to speed.
  EXPECT_TRUE(shooter_queue_.status->ready_left);
  EXPECT_TRUE(shooter_queue_.status->ready_right);
  EXPECT_TRUE(shooter_queue_.status->ready_both);
}

// Test to make sure that it does not exceed the encoder's rated speed.
TEST_F(ShooterTest, RateLimitTest) {
  ASSERT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity_left(1000.0)
                  .angular_velocity_right(1000.0)
                  .Send());

  RunForTime(Time::InSeconds(10));
  EXPECT_TRUE(shooter_queue_.status.FetchLatest());
  EXPECT_TRUE(shooter_queue_.status->ready_both);

  shooter_queue_.goal.FetchLatest();
  shooter_queue_.status.FetchLatest();

  EXPECT_TRUE(shooter_queue_.goal.get() != nullptr);
  EXPECT_TRUE(shooter_queue_.status.get() != nullptr);

  EXPECT_GT(::y2016::control_loops::kMaxSpeed,
            shooter_queue_.status->avg_angular_velocity_left);

  EXPECT_GT(::y2016::control_loops::kMaxSpeed,
            shooter_queue_.status->avg_angular_velocity_right);
}

// Tests that the shooter can spin up nicely while missing position packets.
TEST_F(ShooterTest, MissingPositionMessages) {
  ASSERT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity_left(200.0)
                  .angular_velocity_right(200.0)
                  .Send());
  for (int i = 0; i < 100; ++i) {
    if (i % 7) {
      shooter_plant_.SendPositionMessage();
    }

    RunIteration();
  }

  VerifyNearGoal();
}

// Tests that the shooter can spin up nicely while disabled for a while.
// TODO(comran): Update this test, since it's the same as
// MissingPositionMessages.
TEST_F(ShooterTest, Disabled) {
  ASSERT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity_left(200.0)
                  .angular_velocity_right(200.0)
                  .Send());
  for (int i = 0; i < 100; ++i) {
    if (i % 7) {
      shooter_plant_.SendPositionMessage();
    }

    RunIteration();
  }

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace control_loops
}  // namespace y2016
