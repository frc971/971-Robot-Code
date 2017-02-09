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
#include "y2017/control_loops/superstructure/turret/turret_plant.h"
#include "y2017/control_loops/superstructure/intake/intake_plant.h"

using ::frc971::control_loops::PositionSensorSimulator;

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace testing {
namespace {
constexpr double kNoiseScalar = 0.01;
}  // namespace

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

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

class TurretPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit TurretPlant(StateFeedbackPlant<2, 1, 1> &&other)
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

class IntakePlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit IntakePlant(StateFeedbackPlant<2, 1, 1> &&other)
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

// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation()
      : hood_plant_(new HoodPlant(
            ::y2017::control_loops::superstructure::hood::MakeHoodPlant())),
        hood_pot_encoder_(constants::Values::kHoodEncoderIndexDifference),

        turret_plant_(new TurretPlant(
            ::y2017::control_loops::superstructure::turret::MakeTurretPlant())),
        turret_pot_encoder_(constants::Values::kTurretEncoderIndexDifference),

        intake_plant_(new IntakePlant(
            ::y2017::control_loops::superstructure::intake::MakeIntakePlant())),
        intake_pot_encoder_(constants::Values::kIntakeEncoderIndexDifference),

        superstructure_queue_(".y2017.control_loops.superstructure", 0xdeadbeef,
                              ".y2017.control_loops.superstructure.goal",
                              ".y2017.control_loops.superstructure.position",
                              ".y2017.control_loops.superstructure.output",
                              ".y2017.control_loops.superstructure.status") {
    // Start the hood out in the middle by default.
    InitializeHoodPosition((constants::Values::kHoodRange.lower +
                            constants::Values::kHoodRange.upper) /
                           2.0);

    // Start the turret out in the middle by default.
    InitializeTurretPosition((constants::Values::kTurretRange.lower +
                              constants::Values::kTurretRange.upper) /
                             2.0);

    // Start the intake out in the middle by default.
    InitializeIntakePosition((constants::Values::kIntakeRange.lower +
                              constants::Values::kIntakeRange.upper) /
                             2.0);
  }

  void InitializeHoodPosition(double start_pos) {
    hood_plant_->mutable_X(0, 0) = start_pos;
    hood_plant_->mutable_X(1, 0) = 0.0;

    hood_pot_encoder_.Initialize(
        start_pos, kNoiseScalar,
        constants::GetValues().hood.zeroing.measured_index_position);
  }

  void InitializeTurretPosition(double start_pos) {
    turret_plant_->mutable_X(0, 0) = start_pos;
    turret_plant_->mutable_X(1, 0) = 0.0;

    turret_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues().turret.zeroing.measured_absolute_position);
  }

  void InitializeIntakePosition(double start_pos) {
    intake_plant_->mutable_X(0, 0) = start_pos;
    intake_plant_->mutable_X(1, 0) = 0.0;

    intake_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues().intake.zeroing.measured_absolute_position);
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<SuperstructureQueue::Position> position =
        superstructure_queue_.position.MakeMessage();

    hood_pot_encoder_.GetSensorValues(&position->hood);
    turret_pot_encoder_.GetSensorValues(&position->turret);
    intake_pot_encoder_.GetSensorValues(&position->intake);
    position.Send();
  }

  double hood_position() const { return hood_plant_->X(0, 0); }
  double hood_angular_velocity() const { return hood_plant_->X(1, 0); }

  double turret_position() const { return turret_plant_->X(0, 0); }
  double turret_angular_velocity() const { return turret_plant_->X(1, 0); }

  double intake_position() const { return intake_plant_->X(0, 0); }
  double intake_velocity() const { return intake_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_hood_power_error(double power_error) {
    hood_plant_->set_voltage_offset(power_error);
  }

  void set_turret_power_error(double power_error) {
    turret_plant_->set_voltage_offset(power_error);
  }

  void set_intake_power_error(double power_error) {
    intake_plant_->set_voltage_offset(power_error);
  }

  // Simulates the superstructure for a single timestep.
  void Simulate() {
    EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
    EXPECT_TRUE(superstructure_queue_.status.FetchLatest());

    const double voltage_check_hood =
        (static_cast<hood::Hood::State>(
             superstructure_queue_.status->hood.state) ==
         hood::Hood::State::RUNNING)
            ? superstructure::hood::Hood::kOperatingVoltage
            : superstructure::hood::Hood::kZeroingVoltage;

    const double voltage_check_turret =
        (static_cast<turret::Turret::State>(
             superstructure_queue_.status->turret.state) ==
         turret::Turret::State::RUNNING)
            ? superstructure::turret::Turret::kOperatingVoltage
            : superstructure::turret::Turret::kZeroingVoltage;

    const double voltage_check_intake =
        (static_cast<intake::Intake::State>(
             superstructure_queue_.status->intake.state) ==
         intake::Intake::State::RUNNING)
            ? superstructure::intake::Intake::kOperatingVoltage
            : superstructure::intake::Intake::kZeroingVoltage;

    CHECK_LE(::std::abs(superstructure_queue_.output->voltage_hood),
             voltage_check_hood);

    CHECK_LE(::std::abs(superstructure_queue_.output->voltage_turret),
             voltage_check_turret);

    CHECK_LE(::std::abs(superstructure_queue_.output->voltage_intake),
             voltage_check_intake);

    hood_plant_->mutable_U() << superstructure_queue_.output->voltage_hood +
                                    hood_plant_->voltage_offset();

    turret_plant_->mutable_U() << superstructure_queue_.output->voltage_turret +
                                      turret_plant_->voltage_offset();

    intake_plant_->mutable_U() << superstructure_queue_.output->voltage_intake +
                                      intake_plant_->voltage_offset();

    hood_plant_->Update();
    turret_plant_->Update();
    intake_plant_->Update();

    const double angle_hood = hood_plant_->Y(0, 0);
    const double angle_turret = turret_plant_->Y(0, 0);
    const double position_intake = intake_plant_->Y(0, 0);

    hood_pot_encoder_.MoveTo(angle_hood);
    turret_pot_encoder_.MoveTo(angle_turret);
    intake_pot_encoder_.MoveTo(position_intake);

    EXPECT_GE(angle_hood, constants::Values::kHoodRange.lower_hard);
    EXPECT_LE(angle_hood, constants::Values::kHoodRange.upper_hard);
    EXPECT_GE(angle_turret, constants::Values::kTurretRange.lower_hard);
    EXPECT_LE(angle_turret, constants::Values::kTurretRange.upper_hard);
    EXPECT_GE(position_intake, constants::Values::kIntakeRange.lower_hard);
    EXPECT_LE(position_intake, constants::Values::kIntakeRange.upper_hard);
  }

 private:
  ::std::unique_ptr<HoodPlant> hood_plant_;
  PositionSensorSimulator hood_pot_encoder_;

  ::std::unique_ptr<TurretPlant> turret_plant_;
  PositionSensorSimulator turret_pot_encoder_;

  ::std::unique_ptr<IntakePlant> intake_plant_;
  PositionSensorSimulator intake_pot_encoder_;

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

    EXPECT_NEAR(superstructure_queue_.goal->turret.angle,
                superstructure_queue_.status->turret.position, 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->turret.angle,
                superstructure_plant_.turret_position(), 0.001);

    EXPECT_NEAR(superstructure_queue_.goal->intake.distance,
                superstructure_queue_.status->intake.position, 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->intake.distance,
                superstructure_plant_.intake_position(), 0.001);
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

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory
  // that is no longer valid.
  SuperstructureQueue superstructure_queue_;

  // Create a control loop and simulation.
  Superstructure superstructure_;
  SuperstructureSimulation superstructure_plant_;
};

// Tests that the superstructure does nothing when the goal is zero.
TEST_F(SuperstructureTest, DoesNothing) {
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = 0.2;
    goal->turret.angle = 0.0;
    goal->intake.distance = 0.05;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
}

// Tests that the hood, turret and intake loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  // Set a reasonable goal.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = 0.1;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;

    goal->turret.angle = 0.1;
    goal->turret.profile_params.max_velocity = 1;
    goal->turret.profile_params.max_acceleration = 0.5;

    goal->intake.distance = 0.1;
    goal->intake.profile_params.max_velocity = 1;
    goal->intake.profile_params.max_acceleration = 0.5;

    ASSERT_TRUE(goal.Send());
  }

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the hood, turret and intake loops doesn't try and go beyond the
// physical range of the mechanisms.
TEST_F(SuperstructureTest, RespectsRange) {
  // Set some ridiculous goals to test upper limits.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = 100.0;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;

    goal->turret.angle = 100.0;
    goal->turret.profile_params.max_velocity = 1;
    goal->turret.profile_params.max_acceleration = 0.5;

    goal->intake.distance = 100.0;
    goal->intake.profile_params.max_velocity = 1;
    goal->intake.profile_params.max_acceleration = 0.5;

    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();
  EXPECT_NEAR(constants::Values::kHoodRange.upper,
              superstructure_queue_.status->hood.position, 0.001);

  EXPECT_NEAR(constants::Values::kTurretRange.upper,
              superstructure_queue_.status->turret.position, 0.001);

  EXPECT_NEAR(constants::Values::kIntakeRange.upper,
              superstructure_queue_.status->intake.position, 0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = -100.0;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;

    goal->turret.angle = -100.0;
    goal->turret.profile_params.max_velocity = 1;
    goal->turret.profile_params.max_acceleration = 0.5;

    goal->intake.distance = -100.0;
    goal->intake.profile_params.max_velocity = 1;
    goal->intake.profile_params.max_acceleration = 0.5;

    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();
  EXPECT_NEAR(constants::Values::kHoodRange.lower,
              superstructure_queue_.status->hood.position, 0.001);

  EXPECT_NEAR(constants::Values::kTurretRange.lower,
              superstructure_queue_.status->turret.position, 0.001);

  EXPECT_NEAR(constants::Values::kIntakeRange.lower,
              superstructure_queue_.status->intake.position, 0.001);
}

// Tests that the hood, turret and intake loops zeroes when run for a while.
TEST_F(SuperstructureTest, ZeroTest) {
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;

    goal->turret.angle = constants::Values::kTurretRange.lower;
    goal->turret.profile_params.max_velocity = 1;
    goal->turret.profile_params.max_acceleration = 0.5;

    goal->intake.distance = constants::Values::kIntakeRange.lower;
    goal->intake.profile_params.max_velocity = 1;
    goal->intake.profile_params.max_acceleration = 0.5;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  RunForTime(chrono::seconds(5));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(turret::Turret::State::RUNNING, superstructure_.turret().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
}

TEST_F(SuperstructureTest, LowerHardstopStartup) {
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.lower_hard);

  superstructure_plant_.InitializeTurretPosition(
      constants::Values::kTurretRange.lower_hard);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower_hard);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower;
    goal->turret.angle = constants::Values::kTurretRange.lower;
    goal->intake.distance = constants::Values::kIntakeRange.lower;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, UpperHardstopStartup) {
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.upper_hard);

  superstructure_plant_.InitializeTurretPosition(
      constants::Values::kTurretRange.upper_hard);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper_hard);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.upper;
    goal->turret.angle = constants::Values::kTurretRange.upper;
    goal->intake.distance = constants::Values::kIntakeRange.upper;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.upper);

  superstructure_plant_.InitializeTurretPosition(
      constants::Values::kTurretRange.upper);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.upper - 0.1;
    goal->turret.angle = constants::Values::kTurretRange.upper - 0.1;
    goal->intake.distance = constants::Values::kIntakeRange.upper - 0.1;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(turret::Turret::State::RUNNING, superstructure_.turret().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());

  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));

  EXPECT_EQ(hood::Hood::State::UNINITIALIZED, superstructure_.hood().state());
  EXPECT_EQ(turret::Turret::State::UNINITIALIZED,
            superstructure_.turret().state());
  EXPECT_EQ(intake::Intake::State::UNINITIALIZED,
            superstructure_.intake().state());

  RunForTime(chrono::milliseconds(5000));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(turret::Turret::State::RUNNING, superstructure_.turret().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(SuperstructureTest, DisabledGoalTest) {
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder().Send());
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower + 0.03;
    goal->turret.angle = constants::Values::kTurretRange.lower + 0.03;
    goal->intake.distance = constants::Values::kIntakeRange.lower + 0.03;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::milliseconds(100), false);
  EXPECT_EQ(0.0, superstructure_.hood().goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.turret().goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.intake().goal(0, 0));

  // Now make sure they move correctly
  RunForTime(chrono::seconds(4), true);
  EXPECT_NE(0.0, superstructure_.hood().goal(0, 0));
  EXPECT_NE(0.0, superstructure_.turret().goal(0, 0));
  EXPECT_NE(0.0, superstructure_.intake().goal(0, 0));
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
    goal->turret.angle = constants::Values::kTurretRange.lower;
    goal->intake.distance = constants::Values::kIntakeRange.lower;
    ASSERT_TRUE(goal.Send());
  }

  // Run disabled for 2 seconds
  RunForTime(chrono::seconds(2), false);
  EXPECT_EQ(hood::Hood::State::DISABLED_INITIALIZED,
            superstructure_.hood().state());
  EXPECT_EQ(turret::Turret::State::RUNNING,
            superstructure_.turret().state());
  EXPECT_EQ(intake::Intake::State::RUNNING,
            superstructure_.intake().state());

  superstructure_plant_.set_hood_power_error(1.0);

  RunForTime(chrono::seconds(1), false);

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(turret::Turret::State::RUNNING, superstructure_.turret().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());

  RunForTime(chrono::seconds(2), true);

  VerifyNearGoal();
}

// TODO(austin): Test saturation

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
