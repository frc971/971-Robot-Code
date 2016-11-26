#include "y2015_bot3/control_loops/elevator/elevator.h"

#include <math.h>
#include <unistd.h>

#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "aos/common/controls/control_loop_test.h"
#include "y2015_bot3/control_loops/position_sensor_sim.h"
#include "y2015_bot3/control_loops/elevator/elevator_motor_plant.h"
#include "y2015_bot3/control_loops/elevator/elevator.q.h"

using ::y2015_bot3::control_loops::PositionSensorSimulator;

namespace y2015_bot3 {
namespace control_loops {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;


// TODO(comran+adam): Check these over with Austin, Ben, Brian, and others to
// make sure we didn't forget anything -- especially the zeroing tests!!!

class ElevatorSimulator {
 public:
  ElevatorSimulator()
      : plant_(new StateFeedbackPlant<2, 1, 1>(elevator::MakeElevatorPlant())),
        position_sim_(),
        queue_(".y2015_bot3.control_loops.elevator_queue", 0xca8daa3b,
               ".y2015_bot3.control_loops.elevator_queue.goal",
               ".y2015_bot3.control_loops.elevator_queue.position",
               ".y2015_bot3.control_loops.elevator_queue.output",
               ".y2015_bot3.control_loops.elevator_queue.status") {
    // Initialize the elevator.
    InitializePosition(kElevLowerLimit);
  }

  void InitializePosition(double start_pos) {
    InitializePosition(start_pos, kHallEffectPosition);
  }

  void InitializePosition(double start_pos, double hall_effect_position) {
    plant_->mutable_X(0, 0) = start_pos;
    plant_->mutable_X(1, 0) = 0.0;
    position_sim_.Initialize(start_pos, hall_effect_position);
  }

  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::ElevatorQueue::Position> position =
        queue_.position.MakeMessage();
    position_sim_.GetSensorValues(position.get());
    position.Send();
  }

  void set_acceleration_offset(double acceleration_offset) {
    acceleration_offset_ = acceleration_offset;
  }

  double height() const { return plant_->Y(0, 0); }

  void Simulate() {
    EXPECT_TRUE(queue_.output.FetchLatest());

    plant_->mutable_U() << queue_.output->elevator;

    plant_->Update();
    plant_->mutable_X()(1, 0) += acceleration_offset_ * 0.005;

    const double height = plant_->Y(0, 0);

    position_sim_.MoveTo(height);

    EXPECT_GE(height, kElevLowerHardLimit);
    EXPECT_LE(height, kElevUpperHardLimit);
  }

  void MoveTo(double position) { position_sim_.MoveTo(position); }

  double GetVoltage() const { return plant_->U()(0,0); }

 private:
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> plant_;

  PositionSensorSimulator position_sim_;

  ElevatorQueue queue_;
  double acceleration_offset_ = 0.0;
};

class ElevatorTest : public ::aos::testing::ControlLoopTest {
 protected:
  ElevatorTest()
      : queue_(".y2015_bot3.control_loops.elevator_queue", 0xca8daa3b,
               ".y2015_bot3.control_loops.elevator_queue.goal",
               ".y2015_bot3.control_loops.elevator_queue.position",
               ".y2015_bot3.control_loops.elevator_queue.output",
               ".y2015_bot3.control_loops.elevator_queue.status"),
        elevator_(&queue_),
        plant_() {}

  // Checks to see if fetching position/status from queues does not return null
  // pointers.
  void VerifyNearGoal() {
    queue_.goal.FetchLatest();
    queue_.status.FetchLatest();
    EXPECT_TRUE(queue_.goal.get() != nullptr);
    EXPECT_TRUE(queue_.status.get() != nullptr);
    EXPECT_NEAR(queue_.goal->height, queue_.status->height, 0.001);
    EXPECT_NEAR(queue_.goal->height, plant_.height(), 0.001);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);
    plant_.SendPositionMessage();
    elevator_.Iterate();
    plant_.Simulate();

    TickTime();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const monotonic_clock::duration run_for,
                  bool enabled = true) {
    const auto start_time = monotonic_clock::now();
    while (monotonic_clock::now() < start_time + run_for) {
      RunIteration(enabled);
    }
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to. Otherwise, we will have a pointed to shared memory that
  // is no longer valid.
  ElevatorQueue queue_;

  // Create a control loop and simulation.
  Elevator elevator_;
  ElevatorSimulator plant_;
};

// Tests that the loop does nothing when the goal is zero.
TEST_F(ElevatorTest, DoesNothing) {
  // Set the goals to the starting values. This should theoretically guarantee
  // that the controller does nothing.
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .height(kElevLowerLimit)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .Send());

  // Run a few iterations.
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that the loop can reach a goal.
TEST_F(ElevatorTest, ReachesGoal) {
  // Set a reasonable goal.
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .height(kElevLowerLimit + 0.5)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .Send());

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that the loop doesn't try and go beyond the physical range of the
// mechanisms.
TEST_F(ElevatorTest, RespectsRange) {
  // We're going to send the elevator to -5, which should be significantly too
  // low.
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .height(-5.0)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .Send());

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  queue_.status.FetchLatest();
  EXPECT_NEAR(kElevLowerLimit, queue_.status->height, 0.001);

  // We're going to give the elevator some ridiculously high goal.
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .height(50.0)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .Send());

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  queue_.status.FetchLatest();
  EXPECT_NEAR(kElevUpperLimit, queue_.status->height, 0.001);
}

// Tests that the loop zeroes when run for a while.
TEST_F(ElevatorTest, ZeroTest) {
  queue_.goal.MakeWithBuilder()
      .height(0.5)
      .max_velocity(20)
      .max_acceleration(20)
      .Send();
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that starting at the lower hardstops doesn't cause an abort.
TEST_F(ElevatorTest, LowerHardstopStartup) {
  plant_.InitializePosition(kElevLowerHardLimit);
  queue_.goal.MakeWithBuilder().height(0.4).Send();
  RunForTime(chrono::milliseconds(5000));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(ElevatorTest, UpperHardstopStartup) {
  plant_.InitializePosition(kElevUpperHardLimit);
  queue_.goal.MakeWithBuilder().height(0.4).Send();
  RunForTime(chrono::milliseconds(30000));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(ElevatorTest, ResetTest) {
  plant_.InitializePosition(kElevLowerLimit);
  queue_.goal.MakeWithBuilder().height(0.05).Send();
  RunForTime(chrono::milliseconds(5000));

  EXPECT_EQ(Elevator::RUNNING, elevator_.state());
  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));
  EXPECT_NE(Elevator::RUNNING, elevator_.state());
  RunForTime(chrono::milliseconds(10000));
  EXPECT_EQ(Elevator::RUNNING, elevator_.state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(ElevatorTest, DisabledGoalTest) {
  queue_.goal.MakeWithBuilder().height(0.45).Send();

  RunForTime(chrono::milliseconds(100), false);
  EXPECT_EQ(0.0, elevator_.goal_);

  // Now make sure they move correctly
  RunForTime(chrono::milliseconds(4000), true);
  EXPECT_NE(0.0, elevator_.goal_);
}

// Tests that the elevator zeroing goals don't wind up too far.
TEST_F(ElevatorTest, ElevatorGoalPositiveWindupTest) {
  plant_.InitializePosition(0.0, kHallEffectPosition);
  ASSERT_TRUE(queue_.goal.MakeWithBuilder().height(0.45).Send());

  while (elevator_.state() != Elevator::ZEROING) {
    RunIteration();
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));
  double orig_goal = elevator_.goal_;
  elevator_.goal_ += 100.0;

  RunIteration();
  EXPECT_NEAR(orig_goal, elevator_.goal_, 0.05);

  RunIteration();
  EXPECT_EQ(elevator_.loop_->U(), elevator_.loop_->U_uncapped());

  EXPECT_EQ(elevator_.state(), Elevator::ZEROING);
}

// Tests that the elevator zeroing goals don't wind up too far.
TEST_F(ElevatorTest, ElevatorGoalNegativeWindupTest) {
  plant_.InitializePosition(0.0, kHallEffectPosition);
  ASSERT_TRUE(queue_.goal.MakeWithBuilder().height(0.45).Send());

  while (elevator_.state() != Elevator::ZEROING) {
    RunIteration();
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));
  double orig_goal = elevator_.goal_;
  elevator_.goal_ -= 100.0;

  RunIteration();
  EXPECT_NEAR(orig_goal, elevator_.goal_, 0.05);

  RunIteration();
  EXPECT_EQ(elevator_.loop_->U(), elevator_.loop_->U_uncapped());

  EXPECT_EQ(elevator_.state(), Elevator::ZEROING);
}

// Tests that the loop zeroes when run for a while.
TEST_F(ElevatorTest, ZeroNoGoal) {
  RunForTime(chrono::seconds(5));

  EXPECT_EQ(Elevator::RUNNING, elevator_.state());
}

// Zeroing tests
// TODO(comran+adam): The following tests don't look like they are complete, so
// we will need to get these tests checked & passed before running the elevator
// control loop.

// Tests that starting in the middle zeros correctly.
TEST_F(ElevatorTest, CasualStart) {
  const double start_position = (kElevUpperLimit + kElevLowerLimit) / 2.0;
  plant_.InitializePosition(start_position, kHallEffectPosition);

  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  EXPECT_TRUE(elevator_.CheckZeroed());
  queue_.goal.MakeWithBuilder().height(0.5).Send();

  // Find a reasonable value for the time.
  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that a start position above the upper soft limit zeros correctly.
TEST_F(ElevatorTest, AboveSoftLimitStart) {
  const double start_position = (kElevUpperHardLimit + kElevUpperLimit) / 2.0;
  plant_.InitializePosition(start_position, kHallEffectPosition);
  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  EXPECT_TRUE(elevator_.CheckZeroed());
  queue_.goal.MakeWithBuilder().height(0.5).Send();
  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that a start position below the lower soft limit zeros correctly.
TEST_F(ElevatorTest, BelowSoftLimitStart) {
  const double start_position = (kElevLowerHardLimit + kElevLowerLimit) / 2;
  plant_.InitializePosition(start_position, kHallEffectPosition);
  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  EXPECT_TRUE(elevator_.CheckZeroed());
  queue_.goal.MakeWithBuilder().height(0.4).Send();
  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests it zeroes when we start above the hall effect sensor.
TEST_F(ElevatorTest, OverHallEffect) {
  const double start_position = kHallEffectPosition;
  plant_.InitializePosition(start_position, kHallEffectPosition);
  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  EXPECT_TRUE(elevator_.CheckZeroed());
  queue_.goal.MakeWithBuilder().height(0.4).Send();
  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that we go down fast, up slow, and then get zeroed.
TEST_F(ElevatorTest, DownAndUp) {
  const double start_position = (kElevUpperLimit + kElevLowerLimit) / 2.0;
  plant_.InitializePosition(start_position, kHallEffectPosition);

  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  EXPECT_TRUE(elevator_.CheckZeroed());

  // Make sure we ended off the hall effect.
  queue_.position.FetchLatest();
  EXPECT_TRUE(queue_.position.get() != nullptr);
  EXPECT_FALSE(queue_.position->bottom_hall_effect);
}

// Verify that we can zero while disabled.
TEST_F(ElevatorTest, ZeroWhileDisabled) {
  const double start_position = (kElevUpperLimit + kElevLowerLimit) / 2.0;
  plant_.InitializePosition(start_position, kHallEffectPosition);

  // Running iteration while disabled
  RunIteration(false);
  RunIteration(false);

  // Move elevator below hall effect.
  plant_.MoveTo(kHallEffectPosition - 0.1);
  RunIteration(false);
  plant_.MoveTo(kHallEffectPosition - 0.1);
  RunIteration(false);
  // Make sure it only zeroes while going up.
  EXPECT_TRUE(!elevator_.CheckZeroed());
  // Move above the hall effect.
  plant_.MoveTo(kHallEffectPosition + 0.1);
  RunIteration(false);
  RunIteration(false);
  // Make sure we are zeroed.
  EXPECT_TRUE(elevator_.CheckZeroed());
}

// Tests that the loop can reach a goal with a constant force.
TEST_F(ElevatorTest, ReachesGoalWithIntegral) {
  // Set a reasonable goal.
  ASSERT_TRUE(queue_.goal.MakeWithBuilder()
                  .height(kElevLowerLimit + 0.5)
                  .max_velocity(20)
                  .max_acceleration(20)
                  .Send());

  plant_.set_acceleration_offset(0.1);
  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that the goal (with motion profile) doesn't run away from the elevator
TEST_F(ElevatorTest, PositiveRunawayProfileTest) {
  ASSERT_TRUE(queue_.goal.MakeWithBuilder().height(0.45).Send());

  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));


  double orig_goal = elevator_.goal_;
  {
    Eigen::Matrix<double, 2, 1> current;
    current.setZero();
    current << elevator_.goal_ + 100.0, elevator_.goal_velocity_;
    elevator_.profile_.MoveCurrentState(current);
  }
  RunIteration();
  EXPECT_NEAR(orig_goal, elevator_.goal_, 0.05);

  RunIteration();
  EXPECT_EQ(elevator_.loop_->U(), elevator_.loop_->U_uncapped());

  EXPECT_EQ(elevator_.state(), Elevator::RUNNING);
}

// Tests that the goal (with motion profile) doesn't run away from the elevator
TEST_F(ElevatorTest, NegativeRunawayProfileTest) {
  ASSERT_TRUE(queue_.goal.MakeWithBuilder().height(0.45).Send());

  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  // Kick it.
  RunForTime(chrono::milliseconds(50));


  double orig_goal = elevator_.goal_;
  {
    Eigen::Matrix<double, 2, 1> current;
    current.setZero();
    current << elevator_.goal_ - 100.0, elevator_.goal_velocity_;
    elevator_.profile_.MoveCurrentState(current);
  }
  RunIteration();
  EXPECT_NEAR(orig_goal, elevator_.goal_, 0.05);

  RunIteration();
  EXPECT_EQ(elevator_.loop_->U(), elevator_.loop_->U_uncapped());

  EXPECT_EQ(elevator_.state(), Elevator::RUNNING);
}

// Tests that the glitch filter handles positive edges.
TEST(GlitchFilterTest, PosedgeDetect) {
  GlitchFilter filter;
  filter.Reset(false);
  EXPECT_FALSE(filter.filtered_value());
  EXPECT_FALSE(filter.posedge());

  filter.Update(false, 0.0);
  EXPECT_FALSE(filter.filtered_value());
  EXPECT_FALSE(filter.posedge());

  filter.Update(true, 1.0);
  EXPECT_FALSE(filter.filtered_value());
  EXPECT_FALSE(filter.posedge());

  filter.Update(true, 2.0);
  EXPECT_TRUE(filter.filtered_value());
  EXPECT_TRUE(filter.posedge());

  filter.Update(true, 3.0);
  EXPECT_TRUE(filter.filtered_value());
  EXPECT_FALSE(filter.posedge());
  EXPECT_EQ(0.5, filter.posedge_value());
}

// Tests that the glitch filter handles negative edges.
TEST(GlitchFilterTest, NegedgeDetect) {
  GlitchFilter filter;
  filter.Reset(true);
  EXPECT_TRUE(filter.filtered_value());
  EXPECT_FALSE(filter.negedge());

  filter.Update(true, 0.0);
  EXPECT_TRUE(filter.filtered_value());
  EXPECT_FALSE(filter.negedge());

  filter.Update(false, 1.0);
  EXPECT_TRUE(filter.filtered_value());
  EXPECT_FALSE(filter.negedge());

  filter.Update(false, 2.0);
  EXPECT_FALSE(filter.filtered_value());
  EXPECT_TRUE(filter.negedge());

  filter.Update(false, 3.0);
  EXPECT_FALSE(filter.filtered_value());
  EXPECT_FALSE(filter.negedge());
  EXPECT_EQ(0.5, filter.negedge_value());
}

// Motion Profile Tests


// Makes sure low Accel. and TopSpeed don't output high voltage
TEST_F(ElevatorTest, VeryLowMotionVeryLowVolt) {
  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  // Height guarantees it's not the current position
  queue_.goal.MakeWithBuilder()
      .height(kElevUpperLimit)
      .max_velocity(0.2)
      .max_acceleration(0.2)
      .Send();

  // Checks voltage is not crazy
  // Runs for enough time to move to position
  const auto start_time = monotonic_clock::now();
  const monotonic_clock::duration run_for = chrono::seconds(5);
  // Acceptable range for voltage
  double voltage_range = 5.5;
  while (monotonic_clock::now() < start_time + run_for) {
    RunIteration(true);
    EXPECT_NEAR(0.0, plant_.GetVoltage(), voltage_range);
  }
}

// Makes sure low Accel. and TopSpeed don't output high voltage
TEST_F(ElevatorTest, LowMotionLowVolt) {
  while (elevator_.state() != Elevator::RUNNING) {
    RunIteration();
  }

  // Height guarantees it's not the current position
  queue_.goal.MakeWithBuilder()
      .height(kElevUpperLimit)
      .max_velocity(0.5)
      .max_acceleration(0.5)
      .Send();

  // Checks voltage is not crazy
  // Runs for enough time to move to position
  const auto start_time = monotonic_clock::now();
  const monotonic_clock::duration run_for = chrono::seconds(5);
  // Acceptable range for voltage
  double voltage_range = 7.5;
  while (monotonic_clock::now() < start_time + run_for) {
    RunIteration(true);
    EXPECT_NEAR(0.0, plant_.GetVoltage(), voltage_range);
  }
}

}  // namespace testing
}  // namespace control_loops
}  // namespace y2015_bot3
