#include <math.h>
#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "aos/common/controls/control_loop_test.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/constants.h"
#include "frc971/control_loops/team_number_test_environment.h"

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
        pot_and_encoder_(
            constants::GetValues().claw_zeroing_constants.index_difference),
        claw_queue_(".frc971.control_loops.claw_queue", 0x9d7452fb,
                    ".frc971.control_loops.claw_queue.goal",
                    ".frc971.control_loops.claw_queue.position",
                    ".frc971.control_loops.claw_queue.output",
                    ".frc971.control_loops.claw_queue.status") {
    pot_and_encoder_.Initialize(
        constants::GetValues().claw.wrist.lower_limit,
        constants::GetValues().claw_zeroing_constants.index_difference / 3.0);
  }

  // Do specific initialization for the sensors.
  void SetSensors(double start_pos, double pot_noise_stddev) {
    pot_and_encoder_.Initialize(start_pos, pot_noise_stddev);
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
        claw_plant_() {
    set_team_id(kTeamNumber);
  }

  void VerifyNearGoal() {
    claw_queue_.goal.FetchLatest();
    claw_queue_.status.FetchLatest();
    EXPECT_NEAR(claw_queue_.goal->angle,
                claw_queue_.status->angle,
                10.0);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration() {
    SendMessages(true);

    claw_plant_.SendPositionMessage();
    claw_.Iterate();
    claw_plant_.Simulate();

    TickTime();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const Time &run_for) {
    const auto start_time = Time::Now();
    while (Time::Now() < start_time + run_for) {
      RunIteration();
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
  const auto values = constants::GetValues();
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(values.claw.wrist.lower_limit)
      .Send());

  RunForTime(Time::InMS(4000));

  // We should not have moved.
  VerifyNearGoal();
}

// Tests that we can reach a reasonable goal.
TEST_F(ClawTest, ReachesGoal) {
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(M_PI / 4.0)
      .Send());

  RunForTime(Time::InMS(4000));

  VerifyNearGoal();
}

// Tests that it doesn't try to move past the physical range of the mechanism.
TEST_F(ClawTest, RespectsRange) {
  const auto values = constants::GetValues();
  // Upper limit
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(values.claw.wrist.upper_hard_limit + 5.0)
      .Send());

  RunForTime(Time::InMS(4000));

  claw_queue_.status.FetchLatest();
  /*EXPECT_NEAR(values.claw.wrist.upper_limit,
              claw_queue_.status->angle,
              2.0 * M_PI / 180.0);*/

  // Lower limit.
  ASSERT_TRUE(claw_queue_.goal.MakeWithBuilder()
      .angle(values.claw.wrist.lower_hard_limit + 5.0)
      .Send());

  RunForTime(Time::InMS(4000));

  claw_queue_.status.FetchLatest();
  /*EXPECT_NEAR(values.claw.wrist.lower_limit,
              claw_queue_.status->angle,
              2.0 * M_PI / 180.0);*/
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
