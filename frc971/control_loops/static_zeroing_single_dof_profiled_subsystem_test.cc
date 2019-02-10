#include "gtest/gtest.h"

#include "aos/controls/control_loop_test.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem_test_integral_plant.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem_test_plant.h"

using ::frc971::control_loops::PositionSensorSimulator;

namespace frc971 {
namespace control_loops {
namespace {
constexpr double kNoiseScalar = 0.01;

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

struct TestIntakeSystemValues {
  static const int kZeroingSampleSize = 200;

  static constexpr double kEncoderRatio =
      (16.0 * 0.25) * (20.0 / 40.0) / (2.0 * M_PI) * 0.0254;
  static constexpr double kEncoderIndexDifference = 2.0 * M_PI * kEncoderRatio;
  static constexpr ::frc971::constants::Range kRange{
      .lower_hard = -0.01, .upper_hard = 0.250, .lower = 0.01, .upper = 0.235};

  static constexpr double kZeroingVoltage = 2.5;
  static constexpr double kOperatingVoltage = 12.0;

  static const ::frc971::ProfileParameters kDefaultParams;
  static const ::frc971::ProfileParameters kZeroingParams;

  static constexpr ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
      kZeroing{kZeroingSampleSize, kEncoderIndexDifference, 0, 0.0005, 20, 1.9};

  static const StaticZeroingSingleDOFProfiledSubsystemParams<
      zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      kParams;
};

constexpr ::frc971::constants::Range TestIntakeSystemValues::kRange;
constexpr ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
    TestIntakeSystemValues::kZeroing;
const ::frc971::ProfileParameters TestIntakeSystemValues::kDefaultParams{0.3,
                                                                         5.0};
const ::frc971::ProfileParameters TestIntakeSystemValues::kZeroingParams{0.1,
                                                                         1.0};

const StaticZeroingSingleDOFProfiledSubsystemParams<
    zeroing::PotAndAbsoluteEncoderZeroingEstimator>
    TestIntakeSystemValues::kParams(
        {.zeroing_voltage = TestIntakeSystemValues::kZeroingVoltage,
         .operating_voltage = TestIntakeSystemValues::kOperatingVoltage,
         .zeroing_profile_params = TestIntakeSystemValues::kZeroingParams,
         .default_profile_params = TestIntakeSystemValues::kDefaultParams,
         .range = TestIntakeSystemValues::kRange,
         .zeroing_constants = TestIntakeSystemValues::kZeroing,
         .make_integral_loop = MakeIntegralTestIntakeSystemLoop});

struct TestIntakeSystemData {
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal goal;

  ::frc971::control_loops::AbsoluteProfiledJointStatus status;

  ::frc971::PotAndAbsolutePosition position;

  double output;
};

}  // namespace

class TestIntakeSystemSimulation {
 public:
  TestIntakeSystemSimulation()
      : subsystem_plant_(new CappedTestPlant(
            ::frc971::control_loops::MakeTestIntakeSystemPlant())),
        subsystem_pot_encoder_(
            TestIntakeSystemValues::kEncoderIndexDifference) {
    // Start the subsystem out in the middle by default.
    InitializeSubsystemPosition((TestIntakeSystemValues::kRange.lower +
                                 TestIntakeSystemValues::kRange.upper) /
                                2.0);
  }

  void InitializeSubsystemPosition(double start_pos) {
    subsystem_plant_->mutable_X(0, 0) = start_pos;
    subsystem_plant_->mutable_X(1, 0) = 0.0;

    subsystem_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        TestIntakeSystemValues::kZeroing.measured_absolute_position);
  }

  // Updates the position message with the position of the subsystem.
  void UpdatePositionMessage(::frc971::PotAndAbsolutePosition *position) {
    subsystem_pot_encoder_.GetSensorValues(position);
  }

  double subsystem_position() const { return subsystem_plant_->X(0, 0); }
  double subsystem_velocity() const { return subsystem_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_subsystem_voltage_offset(double voltage_offset) {
    subsystem_plant_->set_voltage_offset(voltage_offset);
  }

  // Simulates the subsystem for a single timestep.
  void Simulate(const double output_voltage, const int32_t state) {
    const double voltage_check_subsystem =
        (static_cast<StaticZeroingSingleDOFProfiledSubsystem<
             zeroing::PotAndAbsoluteEncoderZeroingEstimator>::State>(state) ==
         StaticZeroingSingleDOFProfiledSubsystem<
             zeroing::PotAndAbsoluteEncoderZeroingEstimator>::State::RUNNING)
            ? TestIntakeSystemValues::kOperatingVoltage
            : TestIntakeSystemValues::kZeroingVoltage;

    EXPECT_LE(::std::abs(output_voltage), voltage_check_subsystem);

    ::Eigen::Matrix<double, 1, 1> subsystem_U;
    subsystem_U << output_voltage + subsystem_plant_->voltage_offset();
    subsystem_plant_->Update(subsystem_U);

    const double position_subsystem = subsystem_plant_->Y(0, 0);

    subsystem_pot_encoder_.MoveTo(position_subsystem);

    EXPECT_GE(position_subsystem, TestIntakeSystemValues::kRange.lower_hard);
    EXPECT_LE(position_subsystem, TestIntakeSystemValues::kRange.upper_hard);
  }

 private:
  ::std::unique_ptr<CappedTestPlant> subsystem_plant_;
  PositionSensorSimulator subsystem_pot_encoder_;
};

class IntakeSystemTest : public ::aos::testing::ControlLoopTest {
 protected:
  IntakeSystemTest()
      : subsystem_(TestIntakeSystemValues::kParams), subsystem_plant_() {}

  void VerifyNearGoal() {
    EXPECT_NEAR(subsystem_data_.goal.unsafe_goal,
                subsystem_data_.status.position, 0.001);
    EXPECT_NEAR(subsystem_data_.goal.unsafe_goal,
                subsystem_plant_.subsystem_position(), 0.001);
    EXPECT_NEAR(subsystem_data_.status.velocity, 0, 0.001);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true, bool null_goal = false) {
    SendMessages(enabled);
    subsystem_plant_.UpdatePositionMessage(&subsystem_data_.position);

    // Checks if the robot was reset and resets the subsystem.
    // Required since there is no ControlLoop to reset it (ie. a superstructure)
    ::aos::robot_state.FetchLatest();
    if (::aos::robot_state.get() &&
        sensor_reader_pid_ != ::aos::robot_state->reader_pid) {
      LOG(ERROR, "WPILib reset, restarting\n");
      subsystem_.Reset();
    }

    sensor_reader_pid_ = ::aos::robot_state->reader_pid;
    subsystem_goal_.unsafe_goal = subsystem_data_.goal.unsafe_goal;
    subsystem_goal_.profile_params = subsystem_data_.goal.profile_params;

    subsystem_.Iterate(null_goal ? nullptr : &subsystem_goal_,
                       &subsystem_data_.position, &subsystem_data_.output,
                       &subsystem_data_.status);

    subsystem_plant_.Simulate(subsystem_data_.output,
                              subsystem_data_.status.state);

    TickTime(::std::chrono::microseconds(5050));
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const monotonic_clock::duration run_for, bool enabled = true,
                  bool null_goal = false) {
    const auto start_time = monotonic_clock::now();
    while (monotonic_clock::now() < start_time + run_for) {
      const auto loop_start_time = monotonic_clock::now();
      double begin_subsystem_velocity = subsystem_plant_.subsystem_velocity();

      RunIteration(enabled, null_goal);

      const double loop_time = chrono::duration_cast<chrono::duration<double>>(
                                   monotonic_clock::now() - loop_start_time)
                                   .count();
      const double subsystem_acceleration =
          (subsystem_plant_.subsystem_velocity() - begin_subsystem_velocity) /
          loop_time;
      EXPECT_NEAR(subsystem_acceleration, 0.0, peak_subsystem_acceleration_);
      EXPECT_NEAR(subsystem_plant_.subsystem_velocity(), 0.0,
                  peak_subsystem_velocity_);
    }
  }

  void set_peak_subsystem_acceleration(double value) {
    peak_subsystem_acceleration_ = value;
  }
  void set_peak_subsystem_velocity(double value) {
    peak_subsystem_velocity_ = value;
  }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory
  // that is no longer valid.
  // TestIntakeSystemData subsystem_data_;

  // Create a control loop and simulation.
  StaticZeroingSingleDOFProfiledSubsystem<
      zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      subsystem_;
  TestIntakeSystemSimulation subsystem_plant_;

  StaticZeroingSingleDOFProfiledSubsystemGoal subsystem_goal_;

  TestIntakeSystemData subsystem_data_;

 private:
  // The acceleration limits to check for while moving.
  double peak_subsystem_acceleration_ = 1e10;
  // The velocity limits to check for while moving.
  double peak_subsystem_velocity_ = 1e10;

  int32_t sensor_reader_pid_ = 0;
};

// Tests that the subsystem does nothing when the goal is zero.
TEST_F(IntakeSystemTest, DoesNothing) {
  // Intake system uses 0.05 to test for 0.
  subsystem_data_.goal.unsafe_goal = 0.05;
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that the subsystem loop can reach a goal.
TEST_F(IntakeSystemTest, ReachesGoal) {
  // Set a reasonable goal.
  auto &goal = subsystem_data_.goal;
  goal.unsafe_goal = 0.1;
  goal.profile_params.max_velocity = 1;
  goal.profile_params.max_acceleration = 0.5;

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
TEST_F(IntakeSystemTest, SaturationTest) {
  // Zero it before we move.
  auto &goal = subsystem_data_.goal;
  goal.unsafe_goal = TestIntakeSystemValues::kRange.upper;
  RunForTime(chrono::seconds(8));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    goal.unsafe_goal = TestIntakeSystemValues::kRange.lower;
    goal.profile_params.max_velocity = 20.0;
    goal.profile_params.max_acceleration = 0.1;
  }
  set_peak_subsystem_velocity(23.0);
  set_peak_subsystem_acceleration(0.2);

  RunForTime(chrono::seconds(8));
  VerifyNearGoal();

  // Now do a high acceleration move with a low velocity limit.
  {
    goal.unsafe_goal = TestIntakeSystemValues::kRange.upper;
    goal.profile_params.max_velocity = 0.1;
    goal.profile_params.max_acceleration = 100;
  }

  set_peak_subsystem_velocity(0.2);
  set_peak_subsystem_acceleration(103);
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the subsystem loop doesn't try and go beyond it's physical range
// of the mechanisms.
TEST_F(IntakeSystemTest, RespectsRange) {
  auto &goal = subsystem_data_.goal;
  // Set some ridiculous goals to test upper limits.
  {
    goal.unsafe_goal = 100.0;
    goal.profile_params.max_velocity = 1;
    goal.profile_params.max_acceleration = 0.5;
  }
  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  EXPECT_NEAR(TestIntakeSystemValues::kRange.upper,
              subsystem_data_.status.position, 0.001);

  // Set some ridiculous goals to test lower limits.
  {
    goal.unsafe_goal = -100.0;
    goal.profile_params.max_velocity = 1;
    goal.profile_params.max_acceleration = 0.5;
  }

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  EXPECT_NEAR(TestIntakeSystemValues::kRange.lower,
              subsystem_data_.status.position, 0.001);
}

// Tests that the subsystem loop zeroes when run for a while.
TEST_F(IntakeSystemTest, ZeroTest) {
  auto &goal = subsystem_data_.goal;
  {
    goal.unsafe_goal = TestIntakeSystemValues::kRange.upper;
    goal.profile_params.max_velocity = 1;
    goal.profile_params.max_acceleration = 0.5;
  }

  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(IntakeSystemTest, ZeroNoGoal) {
  RunForTime(chrono::seconds(5));

  EXPECT_EQ(StaticZeroingSingleDOFProfiledSubsystem<
                zeroing::PotAndAbsoluteEncoderZeroingEstimator>::State::RUNNING,
            subsystem_.state());
}

TEST_F(IntakeSystemTest, LowerHardstopStartup) {
  subsystem_plant_.InitializeSubsystemPosition(
      TestIntakeSystemValues::kRange.lower_hard);
  { subsystem_data_.goal.unsafe_goal = TestIntakeSystemValues::kRange.upper; }
  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(IntakeSystemTest, UpperHardstopStartup) {
  subsystem_plant_.InitializeSubsystemPosition(
      TestIntakeSystemValues::kRange.upper_hard);
  { subsystem_data_.goal.unsafe_goal = TestIntakeSystemValues::kRange.upper; }
  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(IntakeSystemTest, ResetTest) {
  subsystem_plant_.InitializeSubsystemPosition(
      TestIntakeSystemValues::kRange.upper);
  {
    subsystem_data_.goal.unsafe_goal =
        TestIntakeSystemValues::kRange.upper - 0.1;
  }
  RunForTime(chrono::seconds(10));

  EXPECT_EQ(StaticZeroingSingleDOFProfiledSubsystem<
                zeroing::PotAndAbsoluteEncoderZeroingEstimator>::State::RUNNING,
            subsystem_.state());

  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));

  EXPECT_EQ(
      StaticZeroingSingleDOFProfiledSubsystem<
          zeroing::PotAndAbsoluteEncoderZeroingEstimator>::State::UNINITIALIZED,
      subsystem_.state());

  RunForTime(chrono::seconds(10));

  EXPECT_EQ(StaticZeroingSingleDOFProfiledSubsystem<
                zeroing::PotAndAbsoluteEncoderZeroingEstimator>::State::RUNNING,
            subsystem_.state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(IntakeSystemTest, DisabledGoalTest) {
  {
    subsystem_data_.goal.unsafe_goal =
        TestIntakeSystemValues::kRange.lower + 0.03;
  }

  // Checks that the subsystem has not moved from its starting position at 0
  RunForTime(chrono::milliseconds(100), false);
  EXPECT_EQ(0.0, subsystem_.goal(0));

  // Now make sure they move correctly
  RunForTime(chrono::seconds(4), true);
  EXPECT_NE(0.0, subsystem_.goal(0));
}

// Tests that zeroing while disabled works.
TEST_F(IntakeSystemTest, DisabledZeroTest) {
  { subsystem_data_.goal.unsafe_goal = TestIntakeSystemValues::kRange.lower; }

  // Run disabled for 2 seconds
  RunForTime(chrono::seconds(2), false);
  EXPECT_EQ(StaticZeroingSingleDOFProfiledSubsystem<
                zeroing::PotAndAbsoluteEncoderZeroingEstimator>::State::RUNNING,
            subsystem_.state());

  RunForTime(chrono::seconds(4), true);

  VerifyNearGoal();
}

// Tests that set_min_position limits range properly
TEST_F(IntakeSystemTest, MinPositionTest) {
  subsystem_data_.goal.unsafe_goal = TestIntakeSystemValues::kRange.lower_hard;
  RunForTime(chrono::seconds(2), true);

  // Check that kRange.lower is used as the default min position
  EXPECT_EQ(subsystem_.goal(0), TestIntakeSystemValues::kRange.lower);
  EXPECT_NEAR(subsystem_data_.status.position,
              TestIntakeSystemValues::kRange.lower, 0.001);

  // Set min position and check that the subsystem increases to that position
  subsystem_.set_min_position(TestIntakeSystemValues::kRange.lower + 0.05);
  RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(subsystem_.goal(0), TestIntakeSystemValues::kRange.lower + 0.05);
  EXPECT_NEAR(subsystem_data_.status.position,
              TestIntakeSystemValues::kRange.lower + 0.05, 0.001);

  // Clear min position and check that the subsystem returns to kRange.lower
  subsystem_.clear_min_position();
  RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(subsystem_.goal(0), TestIntakeSystemValues::kRange.lower);
  EXPECT_NEAR(subsystem_data_.status.position,
              TestIntakeSystemValues::kRange.lower, 0.001);
}

// Tests that set_max_position limits range properly
TEST_F(IntakeSystemTest, MaxPositionTest) {
  subsystem_data_.goal.unsafe_goal = TestIntakeSystemValues::kRange.upper_hard;
  RunForTime(chrono::seconds(2), true);

  // Check that kRange.upper is used as the default max position
  EXPECT_EQ(subsystem_.goal(0), TestIntakeSystemValues::kRange.upper);
  EXPECT_NEAR(subsystem_data_.status.position,
              TestIntakeSystemValues::kRange.upper, 0.001);

  // Set max position and check that the subsystem lowers to that position
  subsystem_.set_max_position(TestIntakeSystemValues::kRange.upper - 0.05);
  RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(subsystem_.goal(0), TestIntakeSystemValues::kRange.upper - 0.05);
  EXPECT_NEAR(subsystem_data_.status.position,
              TestIntakeSystemValues::kRange.upper - 0.05, 0.001);

  // Clear max position and check that the subsystem returns to kRange.upper
  subsystem_.clear_max_position();
  RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(subsystem_.goal(0), TestIntakeSystemValues::kRange.upper);
  EXPECT_NEAR(subsystem_data_.status.position,
              TestIntakeSystemValues::kRange.upper, 0.001);
}

// Tests that the subsystem maintains its current position when sent a null goal
TEST_F(IntakeSystemTest, NullGoalTest) {
  subsystem_data_.goal.unsafe_goal =
      TestIntakeSystemValues::kRange.lower + 0.05;
  RunForTime(chrono::seconds(2), true);

  VerifyNearGoal();

  // Run with a null goal
  RunForTime(chrono::seconds(2), true, true);

  // Check that the subsystem has not moved
  VerifyNearGoal();
}

}  // namespace control_loops
}  // namespace frc971
