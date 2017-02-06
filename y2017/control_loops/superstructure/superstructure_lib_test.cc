#include "y2017/control_loops/superstructure/superstructure.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/common/controls/control_loop_test.h"
#include "aos/common/queue.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/hood/hood_plant.h"

using ::frc971::control_loops::PositionSensorSimulator;

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

// TODO(Adam): Check that the dimensions are correct.
class HoodPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit HoodPlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU() override {
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

// Class which simulates the hood and sends out queue messages with the
// position.
class SuperstructureSimulation {
 public:
  // Constructs a hood simulation.
  static constexpr double kNoiseScalar = 0.01;
  SuperstructureSimulation()
      : hood_plant_(new HoodPlant(
            ::y2017::control_loops::superstructure::hood::MakeHoodPlant())),
        hood_pot_encoder_(constants::Values::kHoodEncoderIndexDifference),
        superstructure_queue_(".y2017.control_loops.superstructure", 0xdeadbeef,
                              ".y2017.control_loops.superstructure.goal",
                              ".y2017.control_loops.superstructure.position",
                              ".y2017.control_loops.superstructure.output",
                              ".y2017.control_loops.superstructure.status") {
    // Start the hood out in the middle by default.
    InitializeHoodPosition((constants::Values::kHoodRange.lower +
                            constants::Values::kHoodRange.upper) /
                           2.0);
  }

  void InitializeHoodPosition(double start_pos) {
    hood_plant_->mutable_X(0, 0) = start_pos;
    hood_plant_->mutable_X(1, 0) = 0.0;

    hood_pot_encoder_.Initialize(
        start_pos, kNoiseScalar,
        constants::GetValues().hood.zeroing.measured_index_position);
  }

  // Sends a queue message with the position of the hood.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<SuperstructureQueue::Position> position =
        superstructure_queue_.position.MakeMessage();

    hood_pot_encoder_.GetSensorValues(&position->hood);
    position.Send();
  }

  double hood_position() const { return hood_plant_->X(0, 0); }

  double hood_angular_velocity() const {
    return hood_plant_->X(1, 0);
  }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_hood_power_error(double power_error) {
    hood_plant_->set_voltage_offset(power_error);
  }

  // Simulates hood for a single timestep.
  void Simulate() {
    EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
    EXPECT_TRUE(superstructure_queue_.status.FetchLatest());

    const double voltage_check =
        (superstructure_queue_.status->hood.state == Superstructure::RUNNING)
            ? superstructure::hood::Hood::kOperatingVoltage
            : superstructure::hood::Hood::kZeroingVoltage;

    CHECK_LE(::std::abs(superstructure_queue_.output->voltage_hood),
             voltage_check);
    hood_plant_->mutable_U()
        << superstructure_queue_.output->voltage_hood +
               hood_plant_->voltage_offset();

    hood_plant_->Update();

    const double angle = hood_plant_->Y(0, 0);
    hood_pot_encoder_.MoveTo(angle);
    EXPECT_GE(angle, constants::Values::kHoodRange.lower_hard);
    EXPECT_LE(angle, constants::Values::kHoodRange.upper_hard);
  }

 private:
  ::std::unique_ptr<HoodPlant> hood_plant_;
  PositionSensorSimulator hood_pot_encoder_;
  SuperstructureQueue superstructure_queue_;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : superstructure_queue_(".y2017.control_loops.superstructure", 0xdeadbeef,
                              ".y2017.control_loops.superstructure.goal",
                              ".y2017.control_loops.superstructure.position",
                              ".y2017.control_loops.superstructure.output",
                              ".y2017.control_loops.superstructure.status"),
        superstructure_(&superstructure_queue_) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    superstructure_queue_.goal.FetchLatest();
    superstructure_queue_.status.FetchLatest();

    ASSERT_TRUE(superstructure_queue_.goal.get() != nullptr);
    ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);

    EXPECT_NEAR(superstructure_queue_.goal->hood.angle,
                superstructure_queue_.status->hood.position, 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->hood.angle,
                superstructure_plant_.hood_position(), 0.001);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    superstructure_plant_.SendPositionMessage();
    superstructure_.Iterate();
    superstructure_plant_.Simulate();

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

  void set_peak_acceleration(double value) { peak_acceleration_ = value; }
  void set_peak_velocity(double value) { peak_velocity_ = value; }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory
  // that is no longer valid.
  SuperstructureQueue superstructure_queue_;

  // Create a control loop and simulation.
  Superstructure superstructure_;
  SuperstructureSimulation superstructure_plant_;

 private:
  double peak_velocity_ = 1e10;
  double peak_acceleration_ = 1e10;
};

// Tests that the hood does nothing when the goal is zero.
TEST_F(SuperstructureTest, DoesNothing) {
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = 0.0;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
}

// Tests that the loop can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  // Set a reasonable goal.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = 0.1;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;
    ASSERT_TRUE(goal.Send());
  }

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the loop doesn't try and go beyond the physical range of the
// mechanisms.
TEST_F(SuperstructureTest, RespectsRange) {
  // Set some ridiculous goals to test upper limits.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = 100.0;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();
  EXPECT_NEAR(constants::Values::kHoodRange.upper,
              superstructure_queue_.status->hood.position, 0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = -100.0;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();
  EXPECT_NEAR(constants::Values::kHoodRange.lower,
              superstructure_queue_.status->hood.position, 0.001);
}

// Tests that the loop zeroes when run for a while.
TEST_F(SuperstructureTest, ZeroTest) {
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  RunForTime(chrono::seconds(5));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
}

TEST_F(SuperstructureTest, LowerHardstopStartup) {
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.lower_hard);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, UpperHardstopStartup) {
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.upper_hard);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.upper;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.upper);

  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.upper - 0.1;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));
  EXPECT_EQ(hood::Hood::State::UNINITIALIZED, superstructure_.hood().state());
  RunForTime(chrono::milliseconds(5000));
  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(SuperstructureTest, DisabledGoalTest) {
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder().Send());
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower + 0.03;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::milliseconds(100), false);
  EXPECT_EQ(0.0, superstructure_.hood().goal(0, 0));

  // Now make sure they move correctly
  RunForTime(chrono::milliseconds(4000), true);
  EXPECT_NE(0.0, superstructure_.hood().goal(0, 0));
}

// Tests that zeroing while disabled works.  Starts the superstructure near a
// pulse, lets it initialize, moves it past the pulse, enables, and then make
// sure it goes to the right spot.
TEST_F(SuperstructureTest, DisabledZeroTest) {
  superstructure_plant_.InitializeHoodPosition(
      constants::GetValues().hood.zeroing.measured_index_position - 0.001);

  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower;
    ASSERT_TRUE(goal.Send());
  }

  // Run disabled for 2 seconds
  RunForTime(chrono::seconds(2), false);
  EXPECT_EQ(hood::Hood::State::DISABLED_INITIALIZED,
            superstructure_.hood().state());

  superstructure_plant_.set_hood_power_error(1.0);

  RunForTime(chrono::seconds(1), false);

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  RunForTime(chrono::seconds(2), true);

  VerifyNearGoal();
}

// TODO(austin): Test saturation

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
