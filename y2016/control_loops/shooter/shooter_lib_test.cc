#include "y2016/control_loops/shooter/shooter.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/controls/control_loop_test.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/control_loops/shooter/shooter.h"
#include "y2016/control_loops/shooter/shooter_plant.h"

using ::frc971::control_loops::testing::kTeamNumber;

namespace y2016 {
namespace control_loops {
namespace shooter {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

class ShooterPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit ShooterPlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 1, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + voltage_offset_);
  }

  double voltage_offset() const { return voltage_offset_; }
  void set_voltage_offset(double voltage_offset) {
    voltage_offset_ = voltage_offset;
  }

 private:
  double voltage_offset_ = 0.0;
};


// Class which simulates the shooter and sends out queue messages with the
// position.
class ShooterSimulation {
 public:
  // Constructs a shooter simulation.
  ShooterSimulation()
      : shooter_plant_left_(new ShooterPlant(
            ::y2016::control_loops::shooter::MakeShooterPlant())),
        shooter_plant_right_(new ShooterPlant(
            ::y2016::control_loops::shooter::MakeShooterPlant())),
        shooter_queue_(".y2016.control_loops.shooter", 0x78d8e372,
                       ".y2016.control_loops.shooter.goal",
                       ".y2016.control_loops.shooter.position",
                       ".y2016.control_loops.shooter.output",
                       ".y2016.control_loops.shooter.status") {}

  // Sends a queue message with the position of the shooter.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<ShooterQueue::Position> position =
        shooter_queue_.position.MakeMessage();

    position->theta_left = shooter_plant_left_->Y(0, 0);
    position->theta_right = shooter_plant_right_->Y(0, 0);
    position.Send();
  }

  void set_left_voltage_offset(double voltage_offset) {
    shooter_plant_left_->set_voltage_offset(voltage_offset);
  }

  void set_right_voltage_offset(double voltage_offset) {
    shooter_plant_right_->set_voltage_offset(voltage_offset);
  }

  // Simulates shooter for a single timestep.
  void Simulate() {
    EXPECT_TRUE(shooter_queue_.output.FetchLatest());

    ::Eigen::Matrix<double, 1, 1> U_left;
    ::Eigen::Matrix<double, 1, 1> U_right;
    U_left(0, 0) = shooter_queue_.output->voltage_left +
                   shooter_plant_left_->voltage_offset();
    U_right(0, 0) = shooter_queue_.output->voltage_right +
                    shooter_plant_right_->voltage_offset();

    shooter_plant_left_->Update(U_left);
    shooter_plant_right_->Update(U_right);
  }

  ::std::unique_ptr<ShooterPlant> shooter_plant_left_, shooter_plant_right_;

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

    EXPECT_NEAR(shooter_queue_.goal->angular_velocity,
                shooter_queue_.status->left.angular_velocity, 10.0);
    EXPECT_NEAR(shooter_queue_.goal->angular_velocity,
                shooter_queue_.status->right.angular_velocity, 10.0);

    EXPECT_NEAR(shooter_queue_.goal->angular_velocity,
                shooter_queue_.status->left.avg_angular_velocity, 10.0);
    EXPECT_NEAR(shooter_queue_.goal->angular_velocity,
                shooter_queue_.status->right.avg_angular_velocity, 10.0);
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
  void RunForTime(const monotonic_clock::duration run_for, bool enabled = true) {
    const auto start_time = monotonic_clock::now();
    while (monotonic_clock::now() < start_time + run_for) {
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
                  .angular_velocity(0.0)
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
  EXPECT_TRUE(
      shooter_queue_.goal.MakeWithBuilder().angular_velocity(450.0).Send());
  RunForTime(chrono::seconds(1));
  VerifyNearGoal();
  EXPECT_TRUE(shooter_queue_.status->ready);
  EXPECT_TRUE(
      shooter_queue_.goal.MakeWithBuilder().angular_velocity(0.0).Send());

  // Make sure we don't apply voltage on spin-down.
  RunIteration();
  EXPECT_TRUE(shooter_queue_.output.FetchLatest());
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_left);
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_right);

  // Continue to stop.
  RunForTime(chrono::seconds(5));
  EXPECT_TRUE(shooter_queue_.output.FetchLatest());
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_left);
  EXPECT_EQ(0.0, shooter_queue_.output->voltage_right);
}

// Tests that the shooter doesn't say it is ready if one side isn't up to speed.
// According to our tuning, we may overshoot the goal on one shooter and
// mistakenly say that we are ready. This test should look at both extremes.
TEST_F(ShooterTest, SideLagTest) {
  // Spin up.
  EXPECT_TRUE(
      shooter_queue_.goal.MakeWithBuilder().angular_velocity(20.0).Send());
  // Cause problems by adding a voltage error on one side.
  shooter_plant_.set_right_voltage_offset(-4.0);
  RunForTime(chrono::milliseconds(100));

  shooter_queue_.goal.FetchLatest();
  shooter_queue_.status.FetchLatest();

  EXPECT_TRUE(shooter_queue_.goal.get() != nullptr);
  EXPECT_TRUE(shooter_queue_.status.get() != nullptr);

  // Left should be up to speed, right shouldn't.
  EXPECT_TRUE(shooter_queue_.status->left.ready);
  EXPECT_FALSE(shooter_queue_.status->right.ready);
  EXPECT_FALSE(shooter_queue_.status->ready);

  RunForTime(chrono::seconds(5));

  shooter_queue_.goal.FetchLatest();
  shooter_queue_.status.FetchLatest();

  EXPECT_TRUE(shooter_queue_.goal.get() != nullptr);
  EXPECT_TRUE(shooter_queue_.status.get() != nullptr);

  // Both should be up to speed.
  EXPECT_TRUE(shooter_queue_.status->left.ready);
  EXPECT_TRUE(shooter_queue_.status->right.ready);
  EXPECT_TRUE(shooter_queue_.status->ready);
}

// Tests that the shooter can spin up nicely after being disabled for a while.
TEST_F(ShooterTest, Disabled) {
  ASSERT_TRUE(shooter_queue_.goal.MakeWithBuilder()
                  .angular_velocity(200.0)
                  .Send());
  RunForTime(chrono::seconds(5), false);
  EXPECT_EQ(nullptr, shooter_queue_.output.get());

  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace shooter
}  // namespace control_loops
}  // namespace y2016
