#include "gtest/gtest.h"

#include "aos/controls/control_loop.h"
#include "aos/controls/control_loop_test.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem_test.q.h"
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

typedef ::testing::Types<
    ::std::pair<
        SZSDPS_AbsEncoder,
        StaticZeroingSingleDOFProfiledSubsystemAbsoluteEncoderTestQueueGroup>,
    ::std::pair<
        SZSDPS_PotAndAbsEncoder,
        StaticZeroingSingleDOFProfiledSubsystemPotAndAbsoluteEncoderTestQueueGroup>>
    TestTypes;

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

}  // namespace

template <typename SZSDPS, typename QueueGroup>
class TestIntakeSystemSimulation {
 public:
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->goal.MakeMessage().get()))>::type
      GoalType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->position.MakeMessage().get()))>::type
      PositionType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->status.MakeMessage().get()))>::type
      StatusType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->output.MakeMessage().get()))>::type
      OutputType;

  TestIntakeSystemSimulation(::aos::EventLoop *event_loop,
                             chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        subsystem_position_sender_(
            event_loop_->MakeSender<PositionType>(".position")),
        subsystem_status_fetcher_(
            event_loop_->MakeFetcher<StatusType>(".status")),
        subsystem_output_fetcher_(
            event_loop_->MakeFetcher<OutputType>(".output")),
        subsystem_plant_(new CappedTestPlant(
            ::frc971::control_loops::MakeTestIntakeSystemPlant())),
        subsystem_sensor_sim_(kEncoderIndexDifference) {
    // Start the subsystem out in the middle by default.
    InitializeSubsystemPosition((kRange.lower + kRange.upper) / 2.0);

    event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            this->Simulate();
          }
          first_ = false;
          this->SendPositionMessage();
        },
        dt);
  }

  void InitializeSubsystemPosition(double start_pos) {
    this->subsystem_plant_->mutable_X(0, 0) = start_pos;
    this->subsystem_plant_->mutable_X(1, 0) = 0.0;

    this->InitializeSensorSim(start_pos);
  }

  void InitializeSensorSim(double start_pos);

  double subsystem_position() const { return this->subsystem_plant_->X(0, 0); }
  double subsystem_velocity() const { return this->subsystem_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_subsystem_voltage_offset(double voltage_offset) {
    this->subsystem_plant_->set_voltage_offset(voltage_offset);
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    typename ::aos::Sender<PositionType>::Message position =
        subsystem_position_sender_.MakeMessage();

    this->subsystem_sensor_sim_.GetSensorValues(&position->position);

    position.Send();
  }

  void set_peak_subsystem_acceleration(double value) {
    peak_subsystem_acceleration_ = value;
  }
  void set_peak_subsystem_velocity(double value) {
    peak_subsystem_velocity_ = value;
  }

  // Simulates the subsystem for a single timestep.
  void Simulate() {
    EXPECT_TRUE(subsystem_output_fetcher_.Fetch());
    EXPECT_TRUE(subsystem_status_fetcher_.Fetch());

    const double begin_subsystem_velocity = subsystem_velocity();

    const double voltage_check_subsystem =
        (static_cast<typename SZSDPS::State>(
             subsystem_status_fetcher_->status.state) == SZSDPS::State::RUNNING)
            ? kOperatingVoltage
            : kZeroingVoltage;

    EXPECT_LE(::std::abs(subsystem_output_fetcher_->output),
              voltage_check_subsystem);

    ::Eigen::Matrix<double, 1, 1> subsystem_U;
    subsystem_U << subsystem_output_fetcher_->output +
                       subsystem_plant_->voltage_offset();
    subsystem_plant_->Update(subsystem_U);

    const double position_subsystem = subsystem_plant_->Y(0, 0);

    subsystem_sensor_sim_.MoveTo(position_subsystem);

    EXPECT_GE(position_subsystem, kRange.lower_hard);
    EXPECT_LE(position_subsystem, kRange.upper_hard);

    const double loop_time = ::aos::time::DurationInSeconds(dt_);
    const double subsystem_acceleration =
        (subsystem_velocity() - begin_subsystem_velocity) / loop_time;
    EXPECT_NEAR(subsystem_acceleration, 0.0, peak_subsystem_acceleration_);
    EXPECT_NEAR(subsystem_velocity(), 0.0, peak_subsystem_velocity_);
  }

 private:
  ::aos::EventLoop *event_loop_;
  chrono::nanoseconds dt_;

  bool first_ = true;

  typename ::aos::Sender<PositionType> subsystem_position_sender_;
  typename ::aos::Fetcher<StatusType> subsystem_status_fetcher_;
  typename ::aos::Fetcher<OutputType> subsystem_output_fetcher_;

  ::std::unique_ptr<CappedTestPlant> subsystem_plant_;
  PositionSensorSimulator subsystem_sensor_sim_;

  // The acceleration limits to check for while moving.
  double peak_subsystem_acceleration_ = 1e10;
  // The velocity limits to check for while moving.
  double peak_subsystem_velocity_ = 1e10;
};

template <>
void TestIntakeSystemSimulation<
    SZSDPS_PotAndAbsEncoder,
    StaticZeroingSingleDOFProfiledSubsystemPotAndAbsoluteEncoderTestQueueGroup>::
    InitializeSensorSim(double start_pos) {
  subsystem_sensor_sim_.Initialize(
      start_pos, kNoiseScalar, 0.0,
      TestIntakeSystemValues<
          typename SZSDPS_PotAndAbsEncoder::ZeroingEstimator>::kZeroing
          .measured_absolute_position);
}

template <>
void TestIntakeSystemSimulation<
    SZSDPS_AbsEncoder,
    StaticZeroingSingleDOFProfiledSubsystemAbsoluteEncoderTestQueueGroup>::
    InitializeSensorSim(double start_pos) {
  subsystem_sensor_sim_.Initialize(
      start_pos, kNoiseScalar, 0.0,
      TestIntakeSystemValues<
          typename SZSDPS_PotAndAbsEncoder::ZeroingEstimator>::kZeroing
          .measured_absolute_position);
}

// Class to represent a module using a subsystem.  This lets us use event loops
// to wrap it.
template <typename QueueGroup, typename SZSDPS>
class Subsystem : public ::aos::controls::ControlLoop<QueueGroup> {
 public:
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->goal.MakeMessage().get()))>::type
      GoalType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->position.MakeMessage().get()))>::type
      PositionType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->status.MakeMessage().get()))>::type
      StatusType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->output.MakeMessage().get()))>::type
      OutputType;

  Subsystem(::aos::EventLoop *event_loop, const ::std::string &name)
      : aos::controls::ControlLoop<QueueGroup>(event_loop, name),
        subsystem_(TestIntakeSystemValues<
                   typename SZSDPS::ZeroingEstimator>::make_params()) {}

  void RunIteration(const GoalType *unsafe_goal, const PositionType *position,
                    OutputType *output, StatusType *status) {
    if (this->WasReset()) {
      LOG(ERROR, "WPILib reset, restarting\n");
      subsystem_.Reset();
    }

    // Convert one goal type to another...
    StaticZeroingSingleDOFProfiledSubsystemGoal goal;
    if (unsafe_goal != nullptr ) {
      goal.unsafe_goal = unsafe_goal->unsafe_goal;
      goal.profile_params.max_velocity =
          unsafe_goal->profile_params.max_velocity;
      goal.profile_params.max_acceleration =
          unsafe_goal->profile_params.max_acceleration;
    }

    subsystem_.Iterate(
        unsafe_goal == nullptr ? nullptr : &goal, &position->position,
        output == nullptr ? nullptr : &output->output, &status->status);
  }

  SZSDPS *subsystem() { return &subsystem_; }

 private:
  SZSDPS subsystem_;
};

template <typename TSZSDPS>
class IntakeSystemTest : public ::aos::testing::ControlLoopTest {
 protected:
  using SZSDPS = typename TSZSDPS::first_type;
  using QueueGroup = typename TSZSDPS::second_type;
  using ZeroingEstimator = typename SZSDPS::ZeroingEstimator;
  using ProfiledJointStatus = typename SZSDPS::ProfiledJointStatus;

  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->goal.MakeMessage().get()))>::type
      GoalType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->position.MakeMessage().get()))>::type
      PositionType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->status.MakeMessage().get()))>::type
      StatusType;
  typedef typename std::remove_reference<decltype(
      *(static_cast<QueueGroup *>(NULL)->output.MakeMessage().get()))>::type
      OutputType;

  IntakeSystemTest()
      : ::aos::testing::ControlLoopTest(chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop()),
        subsystem_goal_sender_(test_event_loop_->MakeSender<GoalType>(".goal")),
        subsystem_goal_fetcher_(
            test_event_loop_->MakeFetcher<GoalType>(".goal")),
        subsystem_status_fetcher_(
            test_event_loop_->MakeFetcher<StatusType>(".status")),
        subsystem_event_loop_(MakeEventLoop()),
        subsystem_(subsystem_event_loop_.get(), ""),
        subsystem_plant_event_loop_(MakeEventLoop()),
        subsystem_plant_(subsystem_plant_event_loop_.get(), dt()) {}

  void VerifyNearGoal() {
    subsystem_goal_fetcher_.Fetch();
    EXPECT_TRUE(subsystem_goal_fetcher_.get() != nullptr);
    EXPECT_TRUE(subsystem_status_fetcher_.Fetch());

    EXPECT_NEAR(subsystem_goal_fetcher_->unsafe_goal,
                subsystem_status_fetcher_->status.position, 0.001);
    EXPECT_NEAR(subsystem_goal_fetcher_->unsafe_goal,
                subsystem_plant_.subsystem_position(), 0.001);
    EXPECT_NEAR(subsystem_status_fetcher_->status.velocity, 0, 0.001);
  }

  SZSDPS *subsystem() { return subsystem_.subsystem(); }

  void set_peak_subsystem_acceleration(double value) {
    set_peak_subsystem_acceleration(value);
  }
  void set_peak_subsystem_velocity(double value) {
    set_peak_subsystem_velocity(value);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Sender<GoalType> subsystem_goal_sender_;
  ::aos::Fetcher<GoalType> subsystem_goal_fetcher_;
  ::aos::Fetcher<StatusType> subsystem_status_fetcher_;

  // Create a control loop and simulation.
  ::std::unique_ptr<::aos::EventLoop> subsystem_event_loop_;
  Subsystem<QueueGroup, SZSDPS> subsystem_;

  ::std::unique_ptr<::aos::EventLoop> subsystem_plant_event_loop_;
  TestIntakeSystemSimulation<SZSDPS, QueueGroup> subsystem_plant_;
};

TYPED_TEST_CASE_P(IntakeSystemTest);

// Tests that the subsystem does nothing when the goal is zero.
TYPED_TEST_P(IntakeSystemTest, DoesNothing) {
  this->SetEnabled(true);
  // Intake system uses 0.05 to test for 0.
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = 0.05;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(5));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop can reach a goal.
TYPED_TEST_P(IntakeSystemTest, ReachesGoal) {
  this->SetEnabled(true);
  // Set a reasonable goal.
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = 0.1;
    message->profile_params.max_velocity = 1;
    message->profile_params.max_acceleration = 0.5;
    EXPECT_TRUE(message.Send());
  }

  // Give it a lot of time to get there.
  this->RunFor(chrono::seconds(8));

  this->VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
TYPED_TEST_P(IntakeSystemTest, SaturationTest) {
  this->SetEnabled(true);
  // Zero it before we move.
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.upper;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(8));
  this->VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.lower;
    message->profile_params.max_velocity = 20.0;
    message->profile_params.max_acceleration = 0.1;
    EXPECT_TRUE(message.Send());
  }
  this->set_peak_subsystem_velocity(23.0);
  this->set_peak_subsystem_acceleration(0.2);

  this->RunFor(chrono::seconds(8));
  this->VerifyNearGoal();

  // Now do a high acceleration move with a low velocity limit.
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.upper;
    message->profile_params.max_velocity = 0.1;
    message->profile_params.max_acceleration = 100;
    EXPECT_TRUE(message.Send());
  }

  this->set_peak_subsystem_velocity(0.2);
  this->set_peak_subsystem_acceleration(103);
  this->RunFor(chrono::seconds(8));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop doesn't try and go beyond it's physical range
// of the mechanisms.
TYPED_TEST_P(IntakeSystemTest, RespectsRange) {
  this->SetEnabled(true);

  // Set some ridiculous goals to test upper limits.
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = 100.0;
    message->profile_params.max_velocity = 1;
    message->profile_params.max_acceleration = 0.5;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.upper, this->subsystem_status_fetcher_->status.position,
              0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = -100.0;
    message->profile_params.max_velocity = 1;
    message->profile_params.max_acceleration = 0.5;
    EXPECT_TRUE(message.Send());
  }

  this->RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.lower, this->subsystem_status_fetcher_->status.position,
              0.001);
}

// Tests that the subsystem loop zeroes when run for a while.
TYPED_TEST_P(IntakeSystemTest, ZeroTest) {
  this->SetEnabled(true);

  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.upper;
    message->profile_params.max_velocity = 1;
    message->profile_params.max_acceleration = 0.5;
    EXPECT_TRUE(message.Send());
  }

  this->RunFor(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TYPED_TEST_P(IntakeSystemTest, ZeroNoGoal) {
  this->SetEnabled(true);
  this->RunFor(chrono::seconds(5));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());
}

TYPED_TEST_P(IntakeSystemTest, LowerHardstopStartup) {
  this->SetEnabled(true);
  this->subsystem_plant_.InitializeSubsystemPosition(kRange.lower_hard);
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.upper;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TYPED_TEST_P(IntakeSystemTest, UpperHardstopStartup) {
  this->SetEnabled(true);

  this->subsystem_plant_.InitializeSubsystemPosition(kRange.upper_hard);
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.upper;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TYPED_TEST_P(IntakeSystemTest, ResetTest) {
  this->SetEnabled(true);

  this->subsystem_plant_.InitializeSubsystemPosition(kRange.upper);
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.upper - 0.1;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(10));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());

  this->VerifyNearGoal();
  this->SimulateSensorReset();
  this->RunFor(chrono::milliseconds(100));

  EXPECT_EQ(TestFixture::SZSDPS::State::UNINITIALIZED,
            this->subsystem()->state());

  this->RunFor(chrono::seconds(10));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());
  this->VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TYPED_TEST_P(IntakeSystemTest, DisabledGoalTest) {
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.lower + 0.03;
    EXPECT_TRUE(message.Send());
  }

  // Checks that the subsystem has not moved from its starting position at 0
  this->RunFor(chrono::milliseconds(100));
  EXPECT_EQ(0.0, this->subsystem()->goal(0));

  // Now make sure they move correctly
  this->SetEnabled(true);
  this->RunFor(chrono::seconds(4));
  EXPECT_NE(0.0, this->subsystem()->goal(0));
}

// Tests that zeroing while disabled works.
TYPED_TEST_P(IntakeSystemTest, DisabledZeroTest) {
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.lower;
    EXPECT_TRUE(message.Send());
  }

  // Run disabled for 2 seconds
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());

  this->SetEnabled(true);
  this->RunFor(chrono::seconds(4));

  this->VerifyNearGoal();
}

// Tests that set_min_position limits range properly
TYPED_TEST_P(IntakeSystemTest, MinPositionTest) {
  this->SetEnabled(true);
  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.lower_hard;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(2));

  // Check that kRange.lower is used as the default min position
  EXPECT_EQ(this->subsystem()->goal(0), kRange.lower);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.lower, this->subsystem_status_fetcher_->status.position,
              0.001);

  // Set min position and check that the subsystem increases to that position
  this->subsystem()->set_min_position(kRange.lower + 0.05);
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), kRange.lower + 0.05);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.lower + 0.05, this->subsystem_status_fetcher_->status.position,
              0.001);

  // Clear min position and check that the subsystem returns to kRange.lower
  this->subsystem()->clear_min_position();
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), kRange.lower);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.lower, this->subsystem_status_fetcher_->status.position,
              0.001);
}

// Tests that set_max_position limits range properly
TYPED_TEST_P(IntakeSystemTest, MaxPositionTest) {
  this->SetEnabled(true);

  {
    auto message = this->subsystem_goal_sender_.MakeMessage();
    message->unsafe_goal = kRange.upper_hard;
    EXPECT_TRUE(message.Send());
  }
  this->RunFor(chrono::seconds(2));

  // Check that kRange.upper is used as the default max position
  EXPECT_EQ(this->subsystem()->goal(0), kRange.upper);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.upper, this->subsystem_status_fetcher_->status.position,
              0.001);

  // Set max position and check that the subsystem lowers to that position
  this->subsystem()->set_max_position(kRange.upper - 0.05);
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), kRange.upper - 0.05);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.upper - 0.05, this->subsystem_status_fetcher_->status.position,
              0.001);

  // Clear max position and check that the subsystem returns to kRange.upper
  this->subsystem()->clear_max_position();
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), kRange.upper);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(kRange.upper, this->subsystem_status_fetcher_->status.position,
              0.001);
}

// Tests that the subsystem maintains its current position when sent a null goal
TYPED_TEST_P(IntakeSystemTest, NullGoalTest) {
  this->SetEnabled(true);

  this->subsystem_plant_.InitializeSubsystemPosition(kRange.upper);

  this->RunFor(chrono::seconds(5));

  EXPECT_NEAR(kRange.upper, this->subsystem_plant_.subsystem_position(), 0.001);
  EXPECT_NEAR(this->subsystem_plant_.subsystem_velocity(), 0, 0.001);
}

// Tests that the subsystem estops when a zeroing error occurs
TYPED_TEST_P(IntakeSystemTest, ZeroingErrorTest) {
  this->SetEnabled(true);
  this->RunFor(chrono::seconds(2));

  EXPECT_EQ(this->subsystem()->state(), TestFixture::SZSDPS::State::RUNNING);
  this->subsystem()->TriggerEstimatorError();
  this->RunFor(this->dt());
  EXPECT_EQ(this->subsystem()->state(), TestFixture::SZSDPS::State::ESTOP);
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
