#include "y2016_bot3/control_loops/intake/intake.h"

#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/queue.h"
#include "aos/common/controls/control_loop_test.h"
#include "aos/common/commonmath.h"
#include "aos/common/time.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "y2016_bot3/control_loops/intake/intake.q.h"
#include "y2016_bot3/control_loops/intake/intake_plant.h"

using ::aos::time::Time;
using ::frc971::control_loops::PositionSensorSimulator;

namespace y2016_bot3 {
namespace control_loops {
namespace intake {
namespace testing {

class IntakePlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit IntakePlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU() override {
    for (int i = 0; i < kNumInputs; ++i) {
      assert(U(i, 0) <= U_max(i, 0) + 0.00001 + voltage_offset_);
      assert(U(i, 0) >= U_min(i, 0) - 0.00001 + voltage_offset_);
    }
  }

  double voltage_offset() const { return voltage_offset_; }
  void set_voltage_offset(double voltage_offset) {
    voltage_offset_ = voltage_offset;
  }

 private:
  double voltage_offset_ = 0.0;
};

// Class which simulates the intake and sends out queue messages with
// the position.
class IntakeSimulation {
 public:
  static constexpr double kNoiseScalar = 0.1;
  IntakeSimulation()
      : intake_plant_(new IntakePlant(MakeIntakePlant())),
        pot_encoder_intake_(
            y2016_bot3::constants::kIntakeEncoderIndexDifference),
        intake_queue_(".y2016_bot3.control_loops.intake", 0x0,
                      ".y2016_bot3.control_loops.intake.goal",
                      ".y2016_bot3.control_loops.intake.status",
                      ".y2016_bot3.control_loops.intake.output",
                      ".y2016_bot3.control_loops.intake.status") {
    InitializeIntakePosition(0.0);
  }

  void InitializeIntakePosition(double start_pos) {
    intake_plant_->mutable_X(0, 0) = start_pos;
    intake_plant_->mutable_X(1, 0) = 0.0;

    pot_encoder_intake_.Initialize(start_pos, kNoiseScalar);
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<control_loops::IntakeQueue::Position> position =
        intake_queue_.position.MakeMessage();

    pot_encoder_intake_.GetSensorValues(&position->intake);

    position.Send();
  }

  double intake_angle() const { return intake_plant_->X(0, 0); }
  double intake_angular_velocity() const { return intake_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_power_error(double power_error_intake) {
    intake_plant_->set_voltage_offset(power_error_intake);
  }

  // Simulates for a single timestep.
  void Simulate() {
    EXPECT_TRUE(intake_queue_.output.FetchLatest());

    // Feed voltages into physics simulation.
    intake_plant_->mutable_U() << intake_queue_.output->voltage_intake +
                                      intake_plant_->voltage_offset();

    // Verify that the correct power limits are being respected depending on
    // which mode we are in.
    EXPECT_TRUE(intake_queue_.status.FetchLatest());
    if (intake_queue_.status->state == Intake::RUNNING) {
      CHECK_LE(::std::abs(intake_queue_.output->voltage_intake),
               Intake::kOperatingVoltage);
    } else {
      CHECK_LE(::std::abs(intake_queue_.output->voltage_intake),
               Intake::kZeroingVoltage);
    }

    // Use the plant to generate the next physical state given the voltages to
    // the motors.
    intake_plant_->Update();

    const double angle_intake = intake_plant_->Y(0, 0);

    // Use the physical state to simulate sensor readings.
    pot_encoder_intake_.MoveTo(angle_intake);

    // Validate that everything is within range.
    EXPECT_GE(angle_intake, y2016_bot3::constants::kIntakeRange.lower_hard);
    EXPECT_LE(angle_intake, y2016_bot3::constants::kIntakeRange.upper_hard);
  }

 private:
  ::std::unique_ptr<IntakePlant> intake_plant_;

  PositionSensorSimulator pot_encoder_intake_;

  IntakeQueue intake_queue_;
};

class IntakeTest : public ::aos::testing::ControlLoopTest {
 protected:
  IntakeTest()
      : intake_queue_(".y2016_bot3.control_loops.intake", 0x0,
                      ".y2016_bot3.control_loops.intake.goal",
                      ".y2016_bot3.control_loops.intake.status",
                      ".y2016_bot3.control_loops.intake.output",
                      ".y2016_bot3.control_loops.intake.status"),
        intake_(&intake_queue_),
        intake_plant_() {}

  void VerifyNearGoal() {
    intake_queue_.goal.FetchLatest();
    intake_queue_.status.FetchLatest();

    EXPECT_TRUE(intake_queue_.goal.get() != nullptr);
    EXPECT_TRUE(intake_queue_.status.get() != nullptr);

    EXPECT_NEAR(intake_queue_.goal->angle_intake,
                intake_queue_.status->intake.angle, 0.001);
    EXPECT_NEAR(intake_queue_.goal->angle_intake, intake_plant_.intake_angle(),
                0.001);
  }

  // Runs one iteration of the whole simulation
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    intake_plant_.SendPositionMessage();
    intake_.Iterate();
    intake_plant_.Simulate();

    TickTime();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const Time &run_for, bool enabled = true) {
    const auto start_time = Time::Now();
    while (Time::Now() < start_time + run_for) {
      const auto loop_start_time = Time::Now();
      double begin_intake_velocity = intake_plant_.intake_angular_velocity();
      RunIteration(enabled);
      const double loop_time = (Time::Now() - loop_start_time).ToSeconds();
      const double intake_acceleration =
          (intake_plant_.intake_angular_velocity() - begin_intake_velocity) /
          loop_time;
      EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
      EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);

      EXPECT_GE(peak_intake_velocity_, intake_plant_.intake_angular_velocity());
      EXPECT_LE(-peak_intake_velocity_,
                intake_plant_.intake_angular_velocity());
    }
  }

  // Runs iterations while watching the average acceleration per cycle and
  // making sure it doesn't exceed the provided bounds.
  void set_peak_intake_acceleration(double value) {
    peak_intake_acceleration_ = value;
  }
  void set_peak_intake_velocity(double value) { peak_intake_velocity_ = value; }



  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  IntakeQueue intake_queue_;

  // Create a control loop and simulation.
  Intake intake_;
  IntakeSimulation intake_plant_;

 private:
  // The acceleration limits to check for while moving for the 3 axes.
  double peak_intake_acceleration_ = 1e10;
  // The velocity limits to check for while moving for the 3 axes.
  double peak_intake_velocity_ = 1e10;
};

// Tests that the intake does nothing when the goal is zero.
TEST_F(IntakeTest, DoesNothing) {
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  // TODO(phil): Send a goal of some sort.
  RunForTime(Time::InSeconds(5));
  VerifyNearGoal();
}

// Tests that the loop can reach a goal.
TEST_F(IntakeTest, ReachesGoal) {
  // Set a reasonable goal.
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(M_PI / 4.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  // Give it a lot of time to get there.
  RunForTime(Time::InSeconds(8));

  VerifyNearGoal();
}

// Tests that the loop doesn't try and go beyond the physical range of the
// mechanisms.
TEST_F(IntakeTest, RespectsRange) {
  // Set some ridiculous goals to test upper limits.
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(M_PI * 10)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());
  RunForTime(Time::InSeconds(10));

  // Check that we are near our soft limit.
  intake_queue_.status.FetchLatest();
  EXPECT_NEAR(y2016_bot3::constants::kIntakeRange.upper,
              intake_queue_.status->intake.angle, 0.001);


  // Set some ridiculous goals to test lower limits.
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(-M_PI * 10)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  RunForTime(Time::InSeconds(10));

  // Check that we are near our soft limit.
  intake_queue_.status.FetchLatest();
  EXPECT_NEAR(y2016_bot3::constants::kIntakeRange.lower,
              intake_queue_.status->intake.angle, 0.001);
}

// Tests that the loop zeroes when run for a while.
TEST_F(IntakeTest, ZeroTest) {
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(y2016_bot3::constants::kIntakeRange.lower)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  RunForTime(Time::InSeconds(10));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(IntakeTest, ZeroNoGoal) {
  RunForTime(Time::InSeconds(5));

  EXPECT_EQ(Intake::RUNNING, intake_.state());
}

// Tests that starting at the lower hardstops doesn't cause an abort.
TEST_F(IntakeTest, LowerHardstopStartup) {
  intake_plant_.InitializeIntakePosition(
      y2016_bot3::constants::kIntakeRange.lower);
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(y2016_bot3::constants::kIntakeRange.upper)
                  .Send());

  RunForTime(Time::InSeconds(15));
  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(IntakeTest, UpperHardstopStartup) {
  intake_plant_.InitializeIntakePosition(
      y2016_bot3::constants::kIntakeRange.upper);
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(y2016_bot3::constants::kIntakeRange.lower)
                  .Send());

  RunForTime(Time::InSeconds(15));
  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(IntakeTest, ResetTest) {
  intake_plant_.InitializeIntakePosition(
      y2016_bot3::constants::kIntakeRange.upper);

  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(y2016_bot3::constants::kIntakeRange.lower + 0.3)
                  .Send());
  RunForTime(Time::InSeconds(15));

  EXPECT_EQ(Intake::RUNNING, intake_.state());
  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(Time::InMS(100));
  EXPECT_NE(Intake::RUNNING, intake_.state());
  RunForTime(Time::InMS(10000));
  EXPECT_EQ(Intake::RUNNING, intake_.state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(IntakeTest, DisabledGoalTest) {
  ASSERT_TRUE(
      intake_queue_.goal.MakeWithBuilder()
          .angle_intake(y2016_bot3::constants::kIntakeRange.lower + 0.03)
          .Send());

  RunForTime(Time::InMS(100), false);
  EXPECT_EQ(0.0, intake_.intake_.goal(0, 0));

  // Now make sure they move correctly
  RunForTime(Time::InMS(4000), true);
  EXPECT_NE(0.0, intake_.intake_.goal(0, 0));
}

// Tests that disabling while zeroing at any state restarts from beginning
TEST_F(IntakeTest, DisabledWhileZeroingHigh) {
  intake_plant_.InitializeIntakePosition(
      y2016_bot3::constants::kIntakeRange.upper);

  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(y2016_bot3::constants::kIntakeRange.upper)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  // Expected states to cycle through and check in order.
  Intake::State kExpectedStateOrder[] = {
      Intake::DISABLED_INITIALIZED, Intake::ZERO_LOWER_INTAKE};

  // Cycle through until intake_ is initialized in intake.cc
  while (intake_.state() < Intake::DISABLED_INITIALIZED) {
    RunIteration(true);
  }

  static const int kNumberOfStates =
      sizeof(kExpectedStateOrder) / sizeof(kExpectedStateOrder[0]);

  // Next state when reached to disable
  for (int i = 0; i < kNumberOfStates; i++) {
    // Next expected state after being disabled that is expected until next
    //  state to disable at is reached
    for (int j = 0; intake_.state() != kExpectedStateOrder[i] && j <= i; j++) {
      // RunIteration until next expected state is reached with a maximum
      //  of 10000 times to ensure a breakout
      for (int o = 0; intake_.state() < kExpectedStateOrder[j] && o < 10000;
           o++) {
        RunIteration(true);
      }
      EXPECT_EQ(kExpectedStateOrder[j], intake_.state());
    }

    EXPECT_EQ(kExpectedStateOrder[i], intake_.state());

    // Disable
    RunIteration(false);

    EXPECT_EQ(Intake::DISABLED_INITIALIZED, intake_.state());
  }

  RunForTime(Time::InSeconds(10));
  EXPECT_EQ(Intake::RUNNING, intake_.state());
}

// Tests that disabling while zeroing at any state restarts from beginning
TEST_F(IntakeTest, DisabledWhileZeroingLow) {
  intake_plant_.InitializeIntakePosition(
      y2016_bot3::constants::kIntakeRange.lower);

  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(y2016_bot3::constants::kIntakeRange.lower)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  // Expected states to cycle through and check in order.
  Intake::State kExpectedStateOrder[] = {
      Intake::DISABLED_INITIALIZED, Intake::ZERO_LIFT_INTAKE};

  // Cycle through until intake_ is initialized in intake.cc
  while (intake_.state() < Intake::DISABLED_INITIALIZED) {
    RunIteration(true);
  }

  static const int kNumberOfStates =
      sizeof(kExpectedStateOrder) / sizeof(kExpectedStateOrder[0]);

  // Next state when reached to disable
  for (int i = 0; i < kNumberOfStates; i++) {
    // Next expected state after being disabled that is expected until next
    //  state to disable at is reached
    for (int j = 0; intake_.state() != kExpectedStateOrder[i] && j <= i; j++) {
      // RunIteration until next expected state is reached with a maximum
      //  of 10000 times to ensure a breakout
      for (int o = 0; intake_.state() < kExpectedStateOrder[j] && o < 10000;
           o++) {
        RunIteration(true);
      }
      EXPECT_EQ(kExpectedStateOrder[j], intake_.state());
    }

    EXPECT_EQ(kExpectedStateOrder[i], intake_.state());

    // Disable
    RunIteration(false);

    EXPECT_EQ(Intake::DISABLED_INITIALIZED, intake_.state());
  }

  RunForTime(Time::InSeconds(10));
  EXPECT_EQ(Intake::RUNNING, intake_.state());
}

// Tests that the integrators works.
TEST_F(IntakeTest, IntegratorTest) {
  intake_plant_.InitializeIntakePosition(
      y2016_bot3::constants::kIntakeRange.lower);
  intake_plant_.set_power_error(1.0);
  intake_queue_.goal.MakeWithBuilder().angle_intake(0.0).Send();

  RunForTime(Time::InSeconds(8));

  VerifyNearGoal();
}

// Tests that zeroing while disabled works.  Starts the superstructure near a
// pulse, lets it initialize, moves it past the pulse, enables, and then make
// sure it goes to the right spot.
TEST_F(IntakeTest, DisabledZeroTest) {
  intake_plant_.InitializeIntakePosition(-0.001);

  intake_queue_.goal.MakeWithBuilder().angle_intake(0.0).Send();

  // Run disabled for 2 seconds
  RunForTime(Time::InSeconds(2), false);
  EXPECT_EQ(Intake::DISABLED_INITIALIZED, intake_.state());

  intake_plant_.set_power_error(1.0);

  RunForTime(Time::InSeconds(1), false);

  EXPECT_EQ(Intake::SLOW_RUNNING, intake_.state());
  RunForTime(Time::InSeconds(2), true);

  VerifyNearGoal();
}

// Tests that the zeroing errors in the intake are caught
TEST_F(IntakeTest, IntakeZeroingErrorTest) {
  RunIteration();
  EXPECT_NE(Intake::ESTOP, intake_.state());
  intake_.intake_.TriggerEstimatorError();
  RunIteration();

  EXPECT_EQ(Intake::ESTOP, intake_.state());
}

// Tests that the loop respects intake acceleration limits while moving.
TEST_F(IntakeTest, IntakeAccelerationLimitTest) {
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  RunForTime(Time::InSeconds(6));
  EXPECT_EQ(Intake::RUNNING, intake_.state());

  VerifyNearGoal();

  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.5)
                  .max_angular_velocity_intake(1)
                  .max_angular_acceleration_intake(1)
                  .Send());

  set_peak_intake_acceleration(1.20);
  RunForTime(Time::InSeconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects intake handles saturation while accelerating
// correctly.
TEST_F(IntakeTest, SaturatedIntakeProfileTest) {
  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .Send());

  RunForTime(Time::InSeconds(6));
  EXPECT_EQ(Intake::RUNNING, intake_.state());

  VerifyNearGoal();

  ASSERT_TRUE(intake_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.5)
                  .max_angular_velocity_intake(4.5)
                  .max_angular_acceleration_intake(800)
                  .Send());

  set_peak_intake_velocity(4.65);
  RunForTime(Time::InSeconds(4));

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace intake
}  // namespace control_loops
}  // namespace frc971
