#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "aos/queue.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2019/constants.h"
#include "y2019/control_loops/superstructure/elevator/elevator_plant.h"
#include "y2019/control_loops/superstructure/intake/intake_plant.h"
#include "y2019/control_loops/superstructure/stilts/stilts_plant.h"
#include "y2019/control_loops/superstructure/superstructure.h"
#include "y2019/control_loops/superstructure/wrist/wrist_plant.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {
namespace testing {

namespace {
constexpr double kNoiseScalar = 0.01;
}  // namespace

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;
using ::frc971::control_loops::PositionSensorSimulator;
using ::frc971::control_loops::CappedTestPlant;
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;
typedef Superstructure::AbsoluteEncoderSubsystem AbsoluteEncoderSubsystem;

// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation()
      : elevator_plant_(
            new CappedTestPlant(::y2019::control_loops::superstructure::
                                    elevator::MakeElevatorPlant())),
        elevator_pot_encoder_(M_PI * 2.0 *
                              constants::Values::kElevatorEncoderRatio()),

        wrist_plant_(new CappedTestPlant(
            ::y2019::control_loops::superstructure::wrist::MakeWristPlant())),
        wrist_pot_encoder_(M_PI * 2.0 *
                           constants::Values::kWristEncoderRatio()),

        intake_plant_(new CappedTestPlant(
            ::y2019::control_loops::superstructure::intake::MakeIntakePlant())),
        intake_pot_encoder_(M_PI * 2.0 *
                            constants::Values::kIntakeEncoderRatio()),

        stilts_plant_(new CappedTestPlant(
            ::y2019::control_loops::superstructure::stilts::MakeStiltsPlant())),
        stilts_pot_encoder_(M_PI * 2.0 *
                            constants::Values::kStiltsEncoderRatio()),

        superstructure_queue_(
            ".y2019.control_loops.superstructure.superstructure_queue",
            ".y2019.control_loops.superstructure.superstructure_queue.goal",
            ".y2019.control_loops.superstructure.superstructure_queue.output",
            ".y2019.control_loops.superstructure.superstructure_queue.status",
            ".y2019.control_loops.superstructure.superstructure_queue."
            "position") {
    // Start the elevator out in the middle by default.
    InitializeElevatorPosition(constants::Values::kElevatorRange().upper);

    // Start the wrist out in the middle by default.
    InitializeWristPosition(constants::Values::kWristRange().upper);

    InitializeIntakePosition(constants::Values::kIntakeRange().upper);

    // Start the stilts out in the middle by default.
    InitializeStiltsPosition(constants::Values::kStiltsRange().lower);
  }

  void InitializeElevatorPosition(double start_pos) {
    elevator_plant_->mutable_X(0, 0) = start_pos;
    elevator_plant_->mutable_X(1, 0) = 0.0;

    elevator_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .elevator.subsystem_params.zeroing_constants
            .measured_absolute_position);
  }

  void InitializeWristPosition(double start_pos) {
    wrist_plant_->mutable_X(0, 0) = start_pos;
    wrist_plant_->mutable_X(1, 0) = 0.0;
    wrist_pot_encoder_.Initialize(start_pos, kNoiseScalar, 0.0,
                                  constants::GetValues()
                                      .wrist.subsystem_params.zeroing_constants
                                      .measured_absolute_position);
  }

  void InitializeIntakePosition(double start_pos) {
    intake_plant_->mutable_X(0, 0) = start_pos;
    intake_plant_->mutable_X(1, 0) = 0.0;

    intake_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .intake.zeroing_constants.measured_absolute_position);
  }

  void InitializeStiltsPosition(double start_pos) {
    stilts_plant_->mutable_X(0, 0) = start_pos;
    stilts_plant_->mutable_X(1, 0) = 0.0;

    stilts_pot_encoder_.Initialize(
        start_pos, kNoiseScalar, 0.0,
        constants::GetValues()
            .stilts.subsystem_params.zeroing_constants
            .measured_absolute_position);
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::ScopedMessagePtr<SuperstructureQueue::Position> position =
        superstructure_queue_.position.MakeMessage();

    elevator_pot_encoder_.GetSensorValues(&position->elevator);
    wrist_pot_encoder_.GetSensorValues(&position->wrist);
    intake_pot_encoder_.GetSensorValues(&position->intake_joint);
    stilts_pot_encoder_.GetSensorValues(&position->stilts);
    position->suction_pressure = simulated_pressure_;

    position.Send();
  }

  double elevator_position() const { return elevator_plant_->X(0, 0); }
  double elevator_velocity() const { return elevator_plant_->X(1, 0); }

  double wrist_position() const { return wrist_plant_->X(0, 0); }
  double wrist_velocity() const { return wrist_plant_->X(1, 0); }

  double intake_position() const { return intake_plant_->X(0, 0); }
  double intake_velocity() const { return intake_plant_->X(1, 0); }

  double stilts_position() const { return stilts_plant_->X(0, 0); }
  double stilts_velocity() const { return stilts_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.

  void set_elevator_voltage_offset(double voltage_offset) {
    elevator_plant_->set_voltage_offset(voltage_offset);
  }

  void set_wrist_voltage_offset(double voltage_offset) {
    wrist_plant_->set_voltage_offset(voltage_offset);
  }

  void set_intake_voltage_offset(double voltage_offset) {
    intake_plant_->set_voltage_offset(voltage_offset);
  }

  void set_stilts_voltage_offset(double voltage_offset) {
    stilts_plant_->set_voltage_offset(voltage_offset);
  }

  void set_simulated_pressure(double pressure) {
    simulated_pressure_ = pressure;
  }

  // Simulates the superstructure for a single timestep.
  void Simulate() {
    EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
    EXPECT_TRUE(superstructure_queue_.status.FetchLatest());

    const double voltage_check_elevator =
        (static_cast<PotAndAbsoluteEncoderSubsystem::State>(
             superstructure_queue_.status->elevator.state) ==
         PotAndAbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().elevator.subsystem_params.operating_voltage
            : constants::GetValues().elevator.subsystem_params.zeroing_voltage;

    const double voltage_check_wrist =
        (static_cast<PotAndAbsoluteEncoderSubsystem::State>(
             superstructure_queue_.status->wrist.state) ==
         PotAndAbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().wrist.subsystem_params.operating_voltage
            : constants::GetValues().wrist.subsystem_params.zeroing_voltage;

    const double voltage_check_intake =
        (static_cast<AbsoluteEncoderSubsystem::State>(
             superstructure_queue_.status->intake.state) ==
         AbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().intake.operating_voltage
            : constants::GetValues().intake.zeroing_voltage;

    const double voltage_check_stilts =
        (static_cast<PotAndAbsoluteEncoderSubsystem::State>(
             superstructure_queue_.status->stilts.state) ==
         PotAndAbsoluteEncoderSubsystem::State::RUNNING)
            ? constants::GetValues().stilts.subsystem_params.operating_voltage
            : constants::GetValues().stilts.subsystem_params.zeroing_voltage;

    EXPECT_NEAR(superstructure_queue_.output->elevator_voltage, 0.0,
                voltage_check_elevator);

    EXPECT_NEAR(superstructure_queue_.output->wrist_voltage, 0.0,
                voltage_check_wrist);

    EXPECT_NEAR(superstructure_queue_.output->intake_joint_voltage, 0.0,
                voltage_check_intake);

    EXPECT_NEAR(superstructure_queue_.output->stilts_voltage, 0.0,
                voltage_check_stilts);

    ::Eigen::Matrix<double, 1, 1> elevator_U;
    elevator_U << superstructure_queue_.output->elevator_voltage +
                      elevator_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> wrist_U;
    wrist_U << superstructure_queue_.output->wrist_voltage +
                   wrist_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> intake_U;
    intake_U << superstructure_queue_.output->intake_joint_voltage +
                    intake_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> stilts_U;
    stilts_U << superstructure_queue_.output->stilts_voltage +
                    stilts_plant_->voltage_offset();

    elevator_plant_->Update(elevator_U);
    wrist_plant_->Update(wrist_U);
    intake_plant_->Update(intake_U);
    stilts_plant_->Update(stilts_U);

    const double position_elevator = elevator_plant_->Y(0, 0);
    const double position_wrist = wrist_plant_->Y(0, 0);
    const double position_intake = intake_plant_->Y(0, 0);
    const double position_stilts = stilts_plant_->Y(0, 0);

    elevator_pot_encoder_.MoveTo(position_elevator);
    wrist_pot_encoder_.MoveTo(position_wrist);
    intake_pot_encoder_.MoveTo(position_intake);
    stilts_pot_encoder_.MoveTo(position_stilts);

    EXPECT_GE(position_elevator,
              constants::Values::kElevatorRange().lower_hard);
    EXPECT_LE(position_elevator,
              constants::Values::kElevatorRange().upper_hard);

    EXPECT_GE(position_wrist, constants::Values::kWristRange().lower_hard);
    EXPECT_LE(position_wrist, constants::Values::kWristRange().upper_hard);

    EXPECT_GE(position_intake, constants::Values::kIntakeRange().lower_hard);
    EXPECT_LE(position_intake, constants::Values::kIntakeRange().upper_hard);

    EXPECT_GE(position_stilts, constants::Values::kStiltsRange().lower_hard);
    EXPECT_LE(position_stilts, constants::Values::kStiltsRange().upper_hard);
  }

 private:
  ::std::unique_ptr<CappedTestPlant> elevator_plant_;
  PositionSensorSimulator elevator_pot_encoder_;

  ::std::unique_ptr<CappedTestPlant> wrist_plant_;
  PositionSensorSimulator wrist_pot_encoder_;

  ::std::unique_ptr<CappedTestPlant> intake_plant_;
  PositionSensorSimulator intake_pot_encoder_;

  ::std::unique_ptr<CappedTestPlant> stilts_plant_;
  PositionSensorSimulator stilts_pot_encoder_;

  double simulated_pressure_ = 1.0;

  SuperstructureQueue superstructure_queue_;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : superstructure_queue_(
            ".y2019.control_loops.superstructure.superstructure_queue",
            ".y2019.control_loops.superstructure.superstructure_queue.goal",
            ".y2019.control_loops.superstructure.superstructure_queue.output",
            ".y2019.control_loops.superstructure.superstructure_queue.status",
            ".y2019.control_loops.superstructure.superstructure_queue."
            "position"),
        superstructure_(&event_loop_) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    superstructure_queue_.goal.FetchLatest();
    superstructure_queue_.status.FetchLatest();

    EXPECT_NEAR(superstructure_queue_.goal->elevator.unsafe_goal,
                superstructure_queue_.status->elevator.position, 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->wrist.unsafe_goal,
                superstructure_plant_.wrist_position(), 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->intake.unsafe_goal,
                superstructure_queue_.status->intake.position, 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->stilts.unsafe_goal,
                superstructure_plant_.stilts_position(), 0.001);
  }

  // Runs one iteration of the whole simulation.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    superstructure_plant_.SendPositionMessage();
    superstructure_.Iterate();
    superstructure_plant_.Simulate();

    TickTime(chrono::microseconds(5050));
  }

  void CheckCollisions() {
    superstructure_queue_.status.FetchLatest();
    ASSERT_FALSE(
        collision_avoidance_.IsCollided(superstructure_queue_.status.get()));
  }

  void WaitUntilZeroed() {
    int i = 0;
    do {
      i++;
      RunIteration();
      superstructure_queue_.status.FetchLatest();
      // 2 Seconds
      ASSERT_LE(i, 2 * 1.0 / .00505);
    } while (!superstructure_queue_.status.get()->zeroed);
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(const monotonic_clock::duration run_for, bool enabled = true,
                  bool check_collisions = true) {
    const auto start_time = monotonic_clock::now();
    while (monotonic_clock::now() < start_time + run_for) {
      const auto loop_start_time = monotonic_clock::now();
      double begin_elevator_velocity =
          superstructure_plant_.elevator_velocity();
      double begin_wrist_velocity = superstructure_plant_.wrist_velocity();
      double begin_intake_velocity = superstructure_plant_.intake_velocity();
      double begin_stilts_velocity = superstructure_plant_.stilts_velocity();

      RunIteration(enabled);
      if (check_collisions) {
        CheckCollisions();
      }

      const double loop_time =
          chrono::duration_cast<chrono::duration<double>>(
              monotonic_clock::now() - loop_start_time).count();

      const double elevator_acceleration =
          (superstructure_plant_.elevator_velocity() -
           begin_elevator_velocity) /
          loop_time;
      const double wrist_acceleration =
          (superstructure_plant_.wrist_velocity() - begin_wrist_velocity) /
          loop_time;
      const double intake_acceleration =
          (superstructure_plant_.intake_velocity() - begin_intake_velocity) /
          loop_time;
      const double stilts_acceleration =
          (superstructure_plant_.stilts_velocity() - begin_stilts_velocity) /
          loop_time;

      EXPECT_GE(peak_elevator_acceleration_, elevator_acceleration);
      EXPECT_LE(-peak_elevator_acceleration_, elevator_acceleration);
      EXPECT_GE(peak_elevator_velocity_,
                superstructure_plant_.elevator_velocity());
      EXPECT_LE(-peak_elevator_velocity_,
                superstructure_plant_.elevator_velocity());

      EXPECT_GE(peak_wrist_acceleration_, wrist_acceleration);
      EXPECT_LE(-peak_wrist_acceleration_, wrist_acceleration);
      EXPECT_GE(peak_wrist_velocity_, superstructure_plant_.wrist_velocity());
      EXPECT_LE(-peak_wrist_velocity_, superstructure_plant_.wrist_velocity());

      EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
      EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);
      EXPECT_GE(peak_intake_velocity_, superstructure_plant_.intake_velocity());
      EXPECT_LE(-peak_intake_velocity_,
                superstructure_plant_.intake_velocity());

      EXPECT_GE(peak_stilts_acceleration_, stilts_acceleration);
      EXPECT_LE(-peak_stilts_acceleration_, stilts_acceleration);
      EXPECT_GE(peak_stilts_velocity_, superstructure_plant_.stilts_velocity());
      EXPECT_LE(-peak_stilts_velocity_,
                superstructure_plant_.stilts_velocity());
    }
  }

  void set_peak_elevator_acceleration(double value) {
    peak_elevator_acceleration_ = value;
  }
  void set_peak_elevator_velocity(double value) {
    peak_elevator_velocity_ = value;
  }

  void set_peak_wrist_acceleration(double value) {
    peak_wrist_acceleration_ = value;
  }
  void set_peak_wrist_velocity(double value) { peak_wrist_velocity_ = value; }

  void set_peak_intake_acceleration(double value) {
    peak_intake_acceleration_ = value;
  }
  void set_peak_intake_velocity(double value) { peak_intake_velocity_ = value; }

  void set_peak_stilts_acceleration(double value) {
    peak_stilts_acceleration_ = value;
  }
  void set_peak_stilts_velocity(double value) { peak_stilts_velocity_ = value; }

  ::aos::ShmEventLoop event_loop_;
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory
  // that is no longer valid.
  SuperstructureQueue superstructure_queue_;

  // Create a control loop and simulation.
  Superstructure superstructure_;
  SuperstructureSimulation superstructure_plant_;

  // Creat a collision avoidance object
  CollisionAvoidance collision_avoidance_;

 private:
  // The acceleration limits to check for while moving.
  double peak_elevator_acceleration_ = 1e10;
  double peak_wrist_acceleration_ = 1e10;
  double peak_intake_acceleration_ = 1e10;
  double peak_stilts_acceleration_ = 1e10;

  // The velocity limits to check for while moving.
  double peak_elevator_velocity_ = 1e10;
  double peak_wrist_velocity_ = 1e10;
  double peak_intake_velocity_ = 1e10;
  double peak_stilts_velocity_ = 1e10;
};

// Tests that the superstructure does nothing when the goal is zero.
TEST_F(SuperstructureTest, DoesNothing) {
  superstructure_plant_.InitializeElevatorPosition(1.4);
  superstructure_plant_.InitializeWristPosition(1.0);
  superstructure_plant_.InitializeIntakePosition(1.1);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  WaitUntilZeroed();

  {
    auto goal = superstructure_queue_.goal.MakeMessage();

    goal->elevator.unsafe_goal = 1.4;
    goal->wrist.unsafe_goal = 1.0;
    goal->intake.unsafe_goal = 1.1;
    goal->stilts.unsafe_goal = 0.1;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));
  VerifyNearGoal();

  EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
}

// Tests that loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  // Set a reasonable goal.

  superstructure_plant_.InitializeElevatorPosition(1.4);
  superstructure_plant_.InitializeWristPosition(1.0);
  superstructure_plant_.InitializeIntakePosition(1.1);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  WaitUntilZeroed();
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->elevator.unsafe_goal = 1.4;
    goal->elevator.profile_params.max_velocity = 1;
    goal->elevator.profile_params.max_acceleration = 0.5;

    goal->wrist.unsafe_goal = 1.0;
    goal->wrist.profile_params.max_velocity = 1;
    goal->wrist.profile_params.max_acceleration = 0.5;

    goal->intake.unsafe_goal = 1.1;
    goal->intake.profile_params.max_velocity = 1;
    goal->intake.profile_params.max_acceleration = 0.5;

    goal->stilts.unsafe_goal = 0.1;
    goal->stilts.profile_params.max_velocity = 1;
    goal->stilts.profile_params.max_acceleration = 0.5;

    ASSERT_TRUE(goal.Send());
  }

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
//
// We are going to disable collision detection to make this easier to implement.
TEST_F(SuperstructureTest, SaturationTest) {
  // Zero it before we move.
  WaitUntilZeroed();
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->elevator.unsafe_goal = constants::Values::kElevatorRange().upper;
    goal->wrist.unsafe_goal = constants::Values::kWristRange().upper;
    goal->intake.unsafe_goal = constants::Values::kIntakeRange().upper;
    goal->stilts.unsafe_goal = constants::Values::kStiltsRange().upper;

    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(8));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->elevator.unsafe_goal = constants::Values::kElevatorRange().upper;
    goal->elevator.profile_params.max_velocity = 20.0;
    goal->elevator.profile_params.max_acceleration = 0.1;

    goal->wrist.unsafe_goal = constants::Values::kWristRange().upper;
    goal->wrist.profile_params.max_velocity = 20.0;
    goal->wrist.profile_params.max_acceleration = 0.1;

    goal->intake.unsafe_goal = constants::Values::kIntakeRange().upper;
    goal->intake.profile_params.max_velocity = 20.0;
    goal->intake.profile_params.max_acceleration = 0.1;

    goal->stilts.unsafe_goal = constants::Values::kStiltsRange().lower;
    goal->stilts.profile_params.max_velocity = 20.0;
    goal->stilts.profile_params.max_acceleration = 0.1;

    ASSERT_TRUE(goal.Send());
  }
  set_peak_elevator_velocity(23.0);
  set_peak_elevator_acceleration(0.2);
  set_peak_wrist_velocity(23.0);
  set_peak_wrist_acceleration(0.2);
  set_peak_intake_velocity(23.0);
  set_peak_intake_acceleration(0.2);
  set_peak_stilts_velocity(23.0);
  set_peak_stilts_acceleration(0.2);

  RunForTime(chrono::seconds(8));
  VerifyNearGoal();

  // Now do a high acceleration move with a low velocity limit.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->elevator.unsafe_goal = constants::Values::kElevatorRange().lower;
    goal->elevator.profile_params.max_velocity = 0.1;
    goal->elevator.profile_params.max_acceleration = 10.0;

    goal->wrist.unsafe_goal = constants::Values::kWristRange().lower;
    goal->wrist.profile_params.max_velocity = 0.1;
    goal->wrist.profile_params.max_acceleration = 10.0;

    goal->intake.unsafe_goal = constants::Values::kIntakeRange().lower;
    goal->intake.profile_params.max_velocity = 0.1;
    goal->intake.profile_params.max_acceleration = 10.0;

    goal->stilts.unsafe_goal = constants::Values::kStiltsRange().lower;
    goal->stilts.profile_params.max_velocity = 0.1;
    goal->stilts.profile_params.max_acceleration = 10.0;
  }
  set_peak_elevator_velocity(0.2);
  set_peak_elevator_acceleration(11.0);
  set_peak_wrist_velocity(0.2);
  set_peak_wrist_acceleration(11.0);
  set_peak_intake_velocity(0.2);
  set_peak_intake_acceleration(11.0);
  set_peak_stilts_velocity(0.2);
  set_peak_stilts_acceleration(11.0);

  VerifyNearGoal();
}

// Tests if the robot zeroes properly... maybe redundant?
TEST_F(SuperstructureTest, ZeroTest) {
  superstructure_plant_.InitializeElevatorPosition(1.4);
  superstructure_plant_.InitializeWristPosition(1.0);
  superstructure_plant_.InitializeIntakePosition(1.1);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  {
    auto goal = superstructure_queue_.goal.MakeMessage();

    goal->elevator.unsafe_goal = 1.4;
    goal->elevator.profile_params.max_velocity = 1;
    goal->elevator.profile_params.max_acceleration = 0.5;

    goal->wrist.unsafe_goal = 1.0;
    goal->wrist.profile_params.max_velocity = 1;
    goal->wrist.profile_params.max_acceleration = 0.5;

    goal->intake.unsafe_goal = 1.1;
    goal->intake.profile_params.max_velocity = 1;
    goal->intake.profile_params.max_acceleration = 0.5;

    goal->stilts.unsafe_goal = 0.1;
    goal->stilts.profile_params.max_velocity = 1;
    goal->stilts.profile_params.max_acceleration = 0.5;

    ASSERT_TRUE(goal.Send());
  }
  WaitUntilZeroed();
  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  WaitUntilZeroed();
  RunForTime(chrono::seconds(2));
  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.elevator().state());

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.wrist().state());

  EXPECT_EQ(AbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.intake().state());

  EXPECT_EQ(PotAndAbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.stilts().state());
}

// Move wrist front to back and see if we collide
TEST_F(SuperstructureTest, CollisionTest) {
  // Set a reasonable goal.
  superstructure_plant_.InitializeElevatorPosition(
      constants::Values::kElevatorRange().lower);
  // 60 degrees forwards
  superstructure_plant_.InitializeWristPosition(M_PI / 3.0);
  superstructure_plant_.InitializeIntakePosition(
      CollisionAvoidance::kIntakeOutAngle + CollisionAvoidance::kEpsIntake);
  superstructure_plant_.InitializeStiltsPosition(0.1);

  WaitUntilZeroed();
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->elevator.unsafe_goal = constants::Values::kElevatorRange().lower;
    goal->wrist.unsafe_goal = -M_PI / 3.0;
    goal->intake.unsafe_goal =
        CollisionAvoidance::kIntakeInAngle - CollisionAvoidance::kEpsIntake;

    ASSERT_TRUE(goal.Send());
  }

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(20), true, true);

  VerifyNearGoal();
}

// Tests that the rollers spin when allowed
TEST_F(SuperstructureTest, IntakeRollerTest) {
  WaitUntilZeroed();

  // Get the elevator and wrist out of the way and set the Intake to where
  // we should be able to spin and verify that they do
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->elevator.unsafe_goal = constants::Values::kElevatorRange().upper;
    goal->wrist.unsafe_goal = 0.0;
    goal->intake.unsafe_goal = constants::Values::kIntakeRange().upper;
    goal->roller_voltage = 6.0;

    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(5), true, true);
  superstructure_queue_.goal.FetchLatest();
  superstructure_queue_.output.FetchLatest();
  EXPECT_EQ(superstructure_queue_.output->intake_roller_voltage,
            superstructure_queue_.goal->roller_voltage);
  VerifyNearGoal();

  // Move the intake where we oughtn't to spin the rollers and verify they don't
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->elevator.unsafe_goal = constants::Values::kElevatorRange().upper;
    goal->wrist.unsafe_goal = 0.0;
    goal->intake.unsafe_goal = constants::Values::kIntakeRange().lower;
    goal->roller_voltage = 6.0;

    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(5), true, true);
  superstructure_queue_.goal.FetchLatest();
  superstructure_queue_.output.FetchLatest();
  EXPECT_EQ(superstructure_queue_.output->intake_roller_voltage, 0.0);
  VerifyNearGoal();
}

// Tests the Vacuum detects a gamepiece
TEST_F(SuperstructureTest, VacuumDetectsPiece) {
  WaitUntilZeroed();
  // Turn on suction
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->suction.top = true;
    goal->suction.bottom = true;

    ASSERT_TRUE(goal.Send());
  }

  RunForTime(
      Vacuum::kTimeAtHigherVoltage - chrono::milliseconds(10),
      true, false);

  // Verify that at 0 pressure after short time voltage is still 12
  superstructure_plant_.set_simulated_pressure(0.0);
  RunForTime(chrono::seconds(2));
  superstructure_queue_.status.FetchLatest();
  EXPECT_TRUE(superstructure_queue_.status->has_piece);
}

// Tests the Vacuum backs off after acquiring a gamepiece
TEST_F(SuperstructureTest, VacuumBacksOff) {
  WaitUntilZeroed();
  // Turn on suction
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->suction.top = true;
    goal->suction.bottom = true;

    ASSERT_TRUE(goal.Send());
  }

  // Verify that at 0 pressure after short time voltage is still high
  superstructure_plant_.set_simulated_pressure(0.0);
  RunForTime(
      Vacuum::kTimeAtHigherVoltage - chrono::milliseconds(10),
      true, false);
  superstructure_queue_.output.FetchLatest();
  EXPECT_EQ(superstructure_queue_.output->pump_voltage, Vacuum::kPumpVoltage);

  // Verify that after waiting with a piece the pump voltage goes to the
  // has piece voltage
  RunForTime(chrono::seconds(2), true, false);
  superstructure_queue_.output.FetchLatest();
  EXPECT_EQ(superstructure_queue_.output->pump_voltage,
            Vacuum::kPumpHasPieceVoltage);
}

// Tests the Vacuum stays on for a bit after getting a no suck goal
TEST_F(SuperstructureTest, VacuumStaysOn) {
  WaitUntilZeroed();
  // Turn on suction
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->suction.top = true;
    goal->suction.bottom = true;

    ASSERT_TRUE(goal.Send());
  }

  // Get a Gamepiece
  superstructure_plant_.set_simulated_pressure(0.0);
  RunForTime(chrono::seconds(2));
  superstructure_queue_.status.FetchLatest();
  EXPECT_TRUE(superstructure_queue_.status->has_piece);

  // Turn off suction
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->suction.top = false;
    goal->suction.bottom = false;
    ASSERT_TRUE(goal.Send());
  }

  superstructure_plant_.set_simulated_pressure(1.0);
  // Run for a short while and make sure we still ask for non-zero volts
  RunForTime(Vacuum::kTimeToKeepPumpRunning -
                 chrono::milliseconds(10),
             true, false);
  superstructure_queue_.output.FetchLatest();
  EXPECT_GE(superstructure_queue_.output->pump_voltage,
            Vacuum::kPumpHasPieceVoltage);

  // Wait and make sure the vacuum actually turns off
  RunForTime(chrono::seconds(2));
  superstructure_queue_.output.FetchLatest();
  EXPECT_EQ(superstructure_queue_.output->pump_voltage, 0.0);
}

// Tests that running disabled, ya know, works
TEST_F(SuperstructureTest, DiasableTest) {
  RunForTime(chrono::seconds(2), false, false);
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
