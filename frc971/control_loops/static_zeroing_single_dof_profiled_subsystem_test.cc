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

using SZSDPS_PotAndAbsEncoder = StaticZeroingSingleDOFProfiledSubsystem<
    zeroing::PotAndAbsoluteEncoderZeroingEstimator,
    ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

using SZSDPS_AbsEncoder = StaticZeroingSingleDOFProfiledSubsystem<
    zeroing::AbsoluteEncoderZeroingEstimator,
    ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

typedef ::testing::Types<SZSDPS_AbsEncoder, SZSDPS_PotAndAbsEncoder> TestTypes;

constexpr ::frc971::constants::Range kRange{
    .lower_hard = -0.01, .upper_hard = 0.250, .lower = 0.01, .upper = 0.235};

constexpr double kZeroingVoltage = 2.5;
constexpr double kOperatingVoltage = 12.0;
const int kZeroingSampleSize = 200;

constexpr double kEncoderIndexDifference = 1.0;

template <typename ZeroingEstimator>
struct TestIntakeSystemValues {
  static const typename ZeroingEstimator::ZeroingConstants kZeroing;

  static const StaticZeroingSingleDOFProfiledSubsystemParams<ZeroingEstimator>
  make_params();
};

template <>
const zeroing::PotAndAbsoluteEncoderZeroingEstimator::ZeroingConstants
    TestIntakeSystemValues<
        zeroing::PotAndAbsoluteEncoderZeroingEstimator>::kZeroing{
        kZeroingSampleSize, kEncoderIndexDifference, 0, 0.0005, 20, 1.9};

template <>
const zeroing::AbsoluteEncoderZeroingEstimator::ZeroingConstants
    TestIntakeSystemValues<zeroing::AbsoluteEncoderZeroingEstimator>::kZeroing{
        kZeroingSampleSize, kEncoderIndexDifference, 0.0, 0.2, 0.0005, 20, 1.9};

template <typename ZeroingEstimator>
const StaticZeroingSingleDOFProfiledSubsystemParams<ZeroingEstimator>
TestIntakeSystemValues<ZeroingEstimator>::make_params() {
  StaticZeroingSingleDOFProfiledSubsystemParams<ZeroingEstimator> params(
      {kZeroingVoltage,
       kOperatingVoltage,
       {0.1, 1.0},
       {0.3, 5.0},
       kRange,
       TestIntakeSystemValues::kZeroing,
       &MakeIntegralTestIntakeSystemLoop});
  return params;
}

template <typename ZeroingEstimator, typename ProfiledJointStatus>
struct TestIntakeSystemData {
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal goal;

  ProfiledJointStatus status;

  typename ZeroingEstimator::Position position;

  double output;
};

}  // namespace

template <typename SZSDPS>
class TestIntakeSystemSimulation {
 public:
  TestIntakeSystemSimulation()
      : subsystem_plant_(new CappedTestPlant(
            ::frc971::control_loops::MakeTestIntakeSystemPlant())),
        subsystem_sensor_sim_(kEncoderIndexDifference) {
    // Start the subsystem out in the middle by default.
    InitializeSubsystemPosition((kRange.lower + kRange.upper) / 2.0);
  }

  void InitializeSubsystemPosition(double start_pos) {
    subsystem_plant_->mutable_X(0, 0) = start_pos;
    subsystem_plant_->mutable_X(1, 0) = 0.0;

    InitializeSensorSim(start_pos);
  }

  void InitializeSensorSim(double start_pos);

  // Updates the position message with the position of the subsystem.
  void UpdatePositionMessage(
      typename SZSDPS::ZeroingEstimator::Position *position) {
    subsystem_sensor_sim_.GetSensorValues(position);
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
        (static_cast<typename SZSDPS::State>(state) == SZSDPS::State::RUNNING)
            ? kOperatingVoltage
            : kZeroingVoltage;

    EXPECT_LE(::std::abs(output_voltage), voltage_check_subsystem);

    ::Eigen::Matrix<double, 1, 1> subsystem_U;
    subsystem_U << output_voltage + subsystem_plant_->voltage_offset();
    subsystem_plant_->Update(subsystem_U);

    const double position_subsystem = subsystem_plant_->Y(0, 0);

    subsystem_sensor_sim_.MoveTo(position_subsystem);

    EXPECT_GE(position_subsystem, kRange.lower_hard);
    EXPECT_LE(position_subsystem, kRange.upper_hard);
  }

 private:
  ::std::unique_ptr<CappedTestPlant> subsystem_plant_;
  PositionSensorSimulator subsystem_sensor_sim_;
};

template <>
void TestIntakeSystemSimulation<SZSDPS_PotAndAbsEncoder>::InitializeSensorSim(
    double start_pos) {
  subsystem_sensor_sim_.Initialize(
      start_pos, kNoiseScalar, 0.0,
      TestIntakeSystemValues<
          typename SZSDPS_PotAndAbsEncoder::ZeroingEstimator>::kZeroing
          .measured_absolute_position);
}

template <>
void TestIntakeSystemSimulation<SZSDPS_AbsEncoder>::InitializeSensorSim(
    double start_pos) {
  subsystem_sensor_sim_.Initialize(
      start_pos, kNoiseScalar, 0.0,
      TestIntakeSystemValues<
          typename SZSDPS_PotAndAbsEncoder::ZeroingEstimator>::kZeroing
          .measured_absolute_position);
}

template <typename TSZSDPS>
class IntakeSystemTest : public ::aos::testing::ControlLoopTest {
 protected:
  using SZSDPS = TSZSDPS;
  using ZeroingEstimator = typename SZSDPS::ZeroingEstimator;
  using ProfiledJointStatus = typename SZSDPS::ProfiledJointStatus;

  IntakeSystemTest()
      : subsystem_(TestIntakeSystemValues<
                   typename SZSDPS::ZeroingEstimator>::make_params()),
        subsystem_plant_() {}

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
  SZSDPS subsystem_;
  TestIntakeSystemSimulation<SZSDPS> subsystem_plant_;

  StaticZeroingSingleDOFProfiledSubsystemGoal subsystem_goal_;

  TestIntakeSystemData<typename SZSDPS::ZeroingEstimator,
                       typename SZSDPS::ProfiledJointStatus>
      subsystem_data_;

 private:
  // The acceleration limits to check for while moving.
  double peak_subsystem_acceleration_ = 1e10;
  // The velocity limits to check for while moving.
  double peak_subsystem_velocity_ = 1e10;

  int32_t sensor_reader_pid_ = 0;
};

TYPED_TEST_CASE_P(IntakeSystemTest);

// Tests that the subsystem does nothing when the goal is zero.
TYPED_TEST_P(IntakeSystemTest, DoesNothing) {
  // Intake system uses 0.05 to test for 0.
  this->subsystem_data_.goal.unsafe_goal = 0.05;
  this->RunForTime(chrono::seconds(5));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop can reach a goal.
TYPED_TEST_P(IntakeSystemTest, ReachesGoal) {
  // Set a reasonable goal.
  auto &goal = this->subsystem_data_.goal;
  goal.unsafe_goal = 0.1;
  goal.profile_params.max_velocity = 1;
  goal.profile_params.max_acceleration = 0.5;

  // Give it a lot of time to get there.
  this->RunForTime(chrono::seconds(8));

  this->VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
TYPED_TEST_P(IntakeSystemTest, SaturationTest) {
  // Zero it before we move.
  auto &goal = this->subsystem_data_.goal;
  goal.unsafe_goal = kRange.upper;
  this->RunForTime(chrono::seconds(8));
  this->VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    goal.unsafe_goal = kRange.lower;
    goal.profile_params.max_velocity = 20.0;
    goal.profile_params.max_acceleration = 0.1;
  }
  this->set_peak_subsystem_velocity(23.0);
  this->set_peak_subsystem_acceleration(0.2);

  this->RunForTime(chrono::seconds(8));
  this->VerifyNearGoal();

  // Now do a high acceleration move with a low velocity limit.
  {
    goal.unsafe_goal = kRange.upper;
    goal.profile_params.max_velocity = 0.1;
    goal.profile_params.max_acceleration = 100;
  }

  this->set_peak_subsystem_velocity(0.2);
  this->set_peak_subsystem_acceleration(103);
  this->RunForTime(chrono::seconds(8));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop doesn't try and go beyond it's physical range
// of the mechanisms.
TYPED_TEST_P(IntakeSystemTest, RespectsRange) {
  auto &goal = this->subsystem_data_.goal;
  // Set some ridiculous goals to test upper limits.
  {
    goal.unsafe_goal = 100.0;
    goal.profile_params.max_velocity = 1;
    goal.profile_params.max_acceleration = 0.5;
  }
  this->RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  EXPECT_NEAR(kRange.upper, this->subsystem_data_.status.position, 0.001);

  // Set some ridiculous goals to test lower limits.
  {
    goal.unsafe_goal = -100.0;
    goal.profile_params.max_velocity = 1;
    goal.profile_params.max_acceleration = 0.5;
  }

  this->RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  EXPECT_NEAR(kRange.lower, this->subsystem_data_.status.position, 0.001);
}

// Tests that the subsystem loop zeroes when run for a while.
TYPED_TEST_P(IntakeSystemTest, ZeroTest) {
  auto &goal = this->subsystem_data_.goal;
  {
    goal.unsafe_goal = kRange.upper;
    goal.profile_params.max_velocity = 1;
    goal.profile_params.max_acceleration = 0.5;
  }

  this->RunForTime(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TYPED_TEST_P(IntakeSystemTest, ZeroNoGoal) {
  this->RunForTime(chrono::seconds(5));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem_.state());
}

TYPED_TEST_P(IntakeSystemTest, LowerHardstopStartup) {
  this->subsystem_plant_.InitializeSubsystemPosition(kRange.lower_hard);
  this->subsystem_data_.goal.unsafe_goal = kRange.upper;
  this->RunForTime(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TYPED_TEST_P(IntakeSystemTest, UpperHardstopStartup) {
  this->subsystem_plant_.InitializeSubsystemPosition(kRange.upper_hard);
  this->subsystem_data_.goal.unsafe_goal = kRange.upper;
  this->RunForTime(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TYPED_TEST_P(IntakeSystemTest, ResetTest) {
  this->subsystem_plant_.InitializeSubsystemPosition(kRange.upper);
  this->subsystem_data_.goal.unsafe_goal = kRange.upper - 0.1;
  this->RunForTime(chrono::seconds(10));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem_.state());

  this->VerifyNearGoal();
  this->SimulateSensorReset();
  this->RunForTime(chrono::milliseconds(100));

  EXPECT_EQ(TestFixture::SZSDPS::State::UNINITIALIZED,
            this->subsystem_.state());

  this->RunForTime(chrono::seconds(10));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem_.state());
  this->VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TYPED_TEST_P(IntakeSystemTest, DisabledGoalTest) {
  this->subsystem_data_.goal.unsafe_goal = kRange.lower + 0.03;

  // Checks that the subsystem has not moved from its starting position at 0
  this->RunForTime(chrono::milliseconds(100), false);
  EXPECT_EQ(0.0, this->subsystem_.goal(0));

  // Now make sure they move correctly
  this->RunForTime(chrono::seconds(4), true);
  EXPECT_NE(0.0, this->subsystem_.goal(0));
}

// Tests that zeroing while disabled works.
TYPED_TEST_P(IntakeSystemTest, DisabledZeroTest) {
  this->subsystem_data_.goal.unsafe_goal = kRange.lower;

  // Run disabled for 2 seconds
  this->RunForTime(chrono::seconds(2), false);
  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem_.state());

  this->RunForTime(chrono::seconds(4), true);

  this->VerifyNearGoal();
}

// Tests that set_min_position limits range properly
TYPED_TEST_P(IntakeSystemTest, MinPositionTest) {
  this->subsystem_data_.goal.unsafe_goal = kRange.lower_hard;
  this->RunForTime(chrono::seconds(2), true);

  // Check that kRange.lower is used as the default min position
  EXPECT_EQ(this->subsystem_.goal(0), kRange.lower);
  EXPECT_NEAR(this->subsystem_data_.status.position, kRange.lower, 0.001);

  // Set min position and check that the subsystem increases to that position
  this->subsystem_.set_min_position(kRange.lower + 0.05);
  this->RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(this->subsystem_.goal(0), kRange.lower + 0.05);
  EXPECT_NEAR(this->subsystem_data_.status.position, kRange.lower + 0.05,
              0.001);

  // Clear min position and check that the subsystem returns to kRange.lower
  this->subsystem_.clear_min_position();
  this->RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(this->subsystem_.goal(0), kRange.lower);
  EXPECT_NEAR(this->subsystem_data_.status.position, kRange.lower, 0.001);
}

// Tests that set_max_position limits range properly
TYPED_TEST_P(IntakeSystemTest, MaxPositionTest) {
  this->subsystem_data_.goal.unsafe_goal = kRange.upper_hard;
  this->RunForTime(chrono::seconds(2), true);

  // Check that kRange.upper is used as the default max position
  EXPECT_EQ(this->subsystem_.goal(0), kRange.upper);
  EXPECT_NEAR(this->subsystem_data_.status.position, kRange.upper, 0.001);

  // Set max position and check that the subsystem lowers to that position
  this->subsystem_.set_max_position(kRange.upper - 0.05);
  this->RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(this->subsystem_.goal(0), kRange.upper - 0.05);
  EXPECT_NEAR(this->subsystem_data_.status.position, kRange.upper - 0.05,
              0.001);

  // Clear max position and check that the subsystem returns to kRange.upper
  this->subsystem_.clear_max_position();
  this->RunForTime(chrono::seconds(2), true);
  EXPECT_EQ(this->subsystem_.goal(0), kRange.upper);
  EXPECT_NEAR(this->subsystem_data_.status.position, kRange.upper, 0.001);
}

// Tests that the subsystem maintains its current position when sent a null goal
TYPED_TEST_P(IntakeSystemTest, NullGoalTest) {
  this->subsystem_data_.goal.unsafe_goal = kRange.lower + 0.05;
  this->RunForTime(chrono::seconds(2), true);

  this->VerifyNearGoal();

  // Run with a null goal
  this->RunForTime(chrono::seconds(2), true, true);

  // Check that the subsystem has not moved
  this->VerifyNearGoal();
}

// Tests that the subsystem estops when a zeroing error occurs
TYPED_TEST_P(IntakeSystemTest, ZeroingErrorTest) {
  this->RunForTime(chrono::seconds(2), true);

  EXPECT_EQ(this->subsystem_.state(), TestFixture::SZSDPS::State::RUNNING);
  this->subsystem_.TriggerEstimatorError();
  this->RunIteration(true, false);
  EXPECT_EQ(this->subsystem_.state(), TestFixture::SZSDPS::State::ESTOP);
}

REGISTER_TYPED_TEST_CASE_P(IntakeSystemTest, DoesNothing, ReachesGoal,
                           SaturationTest, RespectsRange, ZeroTest, ZeroNoGoal,
                           LowerHardstopStartup, UpperHardstopStartup,
                           ResetTest, DisabledGoalTest, DisabledZeroTest,
                           MinPositionTest, MaxPositionTest, NullGoalTest,
                           ZeroingErrorTest);
INSTANTIATE_TYPED_TEST_CASE_P(My, IntakeSystemTest, TestTypes);

}  // namespace control_loops
}  // namespace frc971
