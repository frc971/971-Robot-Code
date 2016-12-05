#include <math.h>
#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "aos/common/controls/control_loop_test.h"
#include "y2015/control_loops/claw/claw.q.h"
#include "y2015/control_loops/claw/claw.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "y2015/constants.h"
#include "frc971/control_loops/team_number_test_environment.h"

using ::aos::time::Time;

namespace y2015 {
namespace control_loops {
namespace testing {

// Class which simulates the claw and sends out queue messages with the
// position.
class ClawSimulation {
 public:
  // Constructs a claw simulation.
  ClawSimulation()
      : claw_plant_(new StateFeedbackPlant<2, 1, 1>(
            y2015::control_loops::claw::MakeClawPlant())),
        pot_and_encoder_(constants::GetValues().claw.zeroing.index_difference),
        claw_queue_(".y2015.control_loops.claw_queue", 0x9d7452fb,
                    ".y2015.control_loops.claw_queue.goal",
                    ".y2015.control_loops.claw_queue.position",
                    ".y2015.control_loops.claw_queue.output",
                    ".y2015.control_loops.claw_queue.status") {
    InitializePosition(constants::GetValues().claw.wrist.lower_limit);
  }

  void InitializePosition(double start_pos) {
    InitializePosition(start_pos,
        constants::GetValues().claw.zeroing.measured_index_position);
  }

  void InitializePosition(double start_pos, double index_pos) {
    InitializePosition(start_pos,
        // This gives us a standard deviation of ~9 degrees on the pot noise.
        constants::GetValues().claw.zeroing.index_difference / 10.0,
        index_pos);
  }

  // Do specific initialization for the sensors.
  void InitializePosition(double start_pos,
                         double pot_noise_stddev,
                         double index_pos) {
    // Update internal state.
    claw_plant_->mutable_X(0, 0) = start_pos;
    // Zero velocity.
    claw_plant_->mutable_X(1, 0) = 0.0;

    pot_and_encoder_.Initialize(start_pos, pot_noise_stddev, index_pos);
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
  ::frc971::control_loops::PositionSensorSimulator pot_and_encoder_;

  ClawQueue claw_queue_;
};

class ClawTest : public ::aos::testing::ControlLoopTest {
 protected:
  ClawTest()
      : claw_queue_(".y2015.control_loops.claw_queue", 0x9d7452fb,
                    ".y2015.control_loops.claw_queue.goal",
                    ".y2015.control_loops.claw_queue.position",
                    ".y2015.control_loops.claw_queue.output",
                    ".y2015.control_loops.claw_queue.status"),
        claw_(&claw_queue_),
        claw_plant_() {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    claw_queue_.goal.FetchLatest();
    claw_queue_.status.FetchLatest();
    EXPECT_NEAR(claw_queue_.goal->angle,
                claw_queue_.status->angle,
                10.0);

    EXPECT_TRUE(claw_queue_.status->zeroed);
    EXPECT_FALSE(claw_queue_.status->estopped);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    claw_plant_.SendPositionMessage();
    claw_.Iterate();
    claw_plant_.Simulate();

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
  ClawQueue claw_queue_;

  // Create a control loop and simulation.
  Claw claw_;
  ClawSimulation claw_plant_;
};

// Tests that the loop does nothing when the goal is our lower limit.
TEST_F(ClawTest, DoesNothing) {
  const auto &values = constants::GetValues();
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(values.claw.wrist.lower_limit)
      .Send());

  RunForTime(Time::InSeconds(6));

  // We should not have moved.
  VerifyNearGoal();
}

// NOTE: Claw zeroing is a little annoying because we only hit one index pulse
// in our entire range of motion.

// Tests that the loop zeroing works with normal values.
TEST_F(ClawTest, Zeroes) {
  // It should zero itself if we run it for awhile.
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  RunForTime(Time::InSeconds(6));

  ASSERT_TRUE(claw_queue_.status.FetchLatest());
  EXPECT_TRUE(claw_queue_.status->zeroed);
  EXPECT_EQ(Claw::RUNNING, claw_queue_.status->state);
}

// Tests that claw zeroing fails if the index pulse occurs too close to the end
// of the range.
TEST_F(ClawTest, BadIndexPosition) {
  const auto values = constants::GetValues();
  claw_plant_.InitializePosition(values.claw.wrist.lower_limit,
                                 values.claw.wrist.upper_limit);

  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  // The zeroing is going to take its sweet time on this one, so we had better
  // run it for longer.
  RunForTime(Time::InMS(12000));

  ASSERT_TRUE(claw_queue_.status.FetchLatest());
  EXPECT_FALSE(claw_queue_.status->zeroed);
  EXPECT_FALSE(claw_queue_.status->estopped);
}

// Tests that we can reach a reasonable goal.
TEST_F(ClawTest, ReachesGoal) {
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  RunForTime(Time::InSeconds(6));

  VerifyNearGoal();
}

// Tests that it doesn't try to move past the physical range of the mechanism.
TEST_F(ClawTest, RespectsRange) {
  const auto &values = constants::GetValues();
  // Upper limit
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(values.claw.wrist.upper_hard_limit + 5.0)
      .Send());

  RunForTime(Time::InSeconds(7));

  claw_queue_.status.FetchLatest();
  EXPECT_NEAR(values.claw.wrist.upper_limit,
              claw_queue_.status->angle,
              2.0 * M_PI / 180.0);

  // Lower limit.
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(values.claw.wrist.lower_hard_limit - 5.0)
      .Send());

  RunForTime(Time::InMS(6000));

  claw_queue_.status.FetchLatest();
  EXPECT_NEAR(values.claw.wrist.lower_limit,
              claw_queue_.status->angle,
              2.0 * M_PI / 180.0);
}

// Tests that starting at the lower hardstop doesn't cause an abort.
TEST_F(ClawTest, LowerHardstopStartup) {
  claw_plant_.InitializePosition(
      constants::GetValues().claw.wrist.lower_hard_limit);
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());
  RunForTime(Time::InSeconds(6));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstop doesn't cause an abort.
TEST_F(ClawTest, UpperHardstopStartup) {
  claw_plant_.InitializePosition(
      constants::GetValues().claw.wrist.upper_hard_limit);
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());
  // Zeroing will take a long time here.
  RunForTime(Time::InSeconds(15));

  VerifyNearGoal();
}


// Tests that not having a goal doesn't break anything.
TEST_F(ClawTest, NoGoal) {
  RunForTime(Time::InMS(50));
}

// Tests that a WPILib reset results in rezeroing.
TEST_F(ClawTest, WpilibReset) {
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  RunForTime(Time::InSeconds(6));
  VerifyNearGoal();

  SimulateSensorReset();
  RunForTime(Time::InMS(100));
  ASSERT_TRUE(claw_queue_.status.FetchLatest());
  EXPECT_NE(Claw::RUNNING, claw_queue_.status->state);

  // Once again, it's going to take us awhile to rezero since we moved away from
  // our index pulse.
  RunForTime(Time::InSeconds(6));
  VerifyNearGoal();
}

// Tests that internal goals don't change while disabled.
TEST_F(ClawTest, DisabledGoal) {
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  RunForTime(Time::InMS(100), false);
  EXPECT_EQ(0.0, claw_.claw_goal_);

  // Now make sure they move correctly.
  RunForTime(Time::InMS(1000), true);
  EXPECT_NE(0.0, claw_.claw_goal_);
}

// Tests that the claw zeroing goals don't wind up too far.
TEST_F(ClawTest, GoalPositiveWindup) {
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  while (claw_.state() != Claw::ZEROING) {
    RunIteration();
  }

  // Kick it.
  RunForTime(Time::InMS(50));
  const double orig_claw_goal = claw_.claw_goal_;
  claw_.claw_goal_ += 100.0;

  RunIteration();
  EXPECT_NEAR(orig_claw_goal, claw_.claw_goal_, 0.10);

  RunIteration();

  EXPECT_EQ(claw_.claw_loop_->U(), claw_.claw_loop_->U_uncapped());
}

// Tests that the claw zeroing goals don't wind up too far.
TEST_F(ClawTest, GoalNegativeWindup) {
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  while (claw_.state() != Claw::ZEROING) {
    RunIteration();
  }

  // Kick it.
  RunForTime(Time::InMS(50));
  double orig_claw_goal = claw_.claw_goal_;
  claw_.claw_goal_ -= 100.0;

  RunIteration();
  EXPECT_NEAR(orig_claw_goal, claw_.claw_goal_, 0.10);

  RunIteration();

  EXPECT_EQ(claw_.claw_loop_->U(), claw_.claw_loop_->U_uncapped());
}

}  // namespace testing
}  // namespace control_loops
}  // namespace y2015
