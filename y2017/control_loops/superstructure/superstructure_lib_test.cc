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
#include "y2017/control_loops/superstructure/column/column_plant.h"
#include "y2017/control_loops/superstructure/hood/hood_plant.h"
#include "y2017/control_loops/superstructure/intake/intake_plant.h"
#include "y2017/control_loops/superstructure/shooter/shooter_plant.h"

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

class ShooterPlant : public StateFeedbackPlant<3, 1, 1> {
 public:
  explicit ShooterPlant(StateFeedbackPlant<3, 1, 1> &&other)
      : StateFeedbackPlant<3, 1, 1>(::std::move(other)) {}

  void CheckU(const Eigen::Matrix<double, 1, 1> &U) override {
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

class HoodPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit HoodPlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 1, 1> &U) override {
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

class ColumnPlant : public StateFeedbackPlant<4, 2, 2> {
 public:
  explicit ColumnPlant(StateFeedbackPlant<4, 2, 2> &&other)
      : StateFeedbackPlant<4, 2, 2>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 2, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + indexer_voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + indexer_voltage_offset_);

    EXPECT_LE(U(1, 0), U_max(1, 0) + 0.00001 + turret_voltage_offset_);
    EXPECT_GE(U(1, 0), U_min(1, 0) - 0.00001 + turret_voltage_offset_);
  }

  double indexer_voltage_offset() const { return indexer_voltage_offset_; }
  void set_indexer_voltage_offset(double indexer_voltage_offset) {
    indexer_voltage_offset_ = indexer_voltage_offset;
  }
  double turret_voltage_offset() const { return turret_voltage_offset_; }
  void set_turret_voltage_offset(double turret_voltage_offset) {
    turret_voltage_offset_ = turret_voltage_offset;
  }

 private:
  double indexer_voltage_offset_ = 0.0;
  double turret_voltage_offset_ = 0.0;
};

class IntakePlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit IntakePlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 1, 1> &U) override {
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
        hood_encoder_(constants::Values::kHoodEncoderIndexDifference),

        intake_plant_(new IntakePlant(
            ::y2017::control_loops::superstructure::intake::MakeIntakePlant())),
        intake_pot_encoder_(constants::Values::kIntakeEncoderIndexDifference),

        shooter_plant_(new ShooterPlant(::y2017::control_loops::superstructure::
                                            shooter::MakeShooterPlant())),

        indexer_encoder_(2.0 * M_PI),
        turret_encoder_(2.0 * M_PI),
        column_plant_(new ColumnPlant(
            ::y2017::control_loops::superstructure::column::MakeColumnPlant())),

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
    InitializeIndexerPosition(0.0);

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

    hood_encoder_.Initialize(
        start_pos, kNoiseScalar,
        constants::GetValues().hood.zeroing.measured_index_position);
  }

  void InitializeIndexerPosition(double start_pos) {
    column_plant_->mutable_X(0, 0) = start_pos;
    column_plant_->mutable_X(1, 0) = 0.0;

    indexer_encoder_.InitializeHallEffectAndPosition(
        start_pos,
        constants::GetValues().column.indexer_zeroing.lower_hall_position,
        constants::GetValues().column.indexer_zeroing.upper_hall_position);

    // Reset the turret encoder too (to the current position), since it has the
    // potential to change.
    InitializeTurretPosition(column_plant_->X(2, 0));
  }

  void InitializeTurretPosition(double start_pos) {
    column_plant_->mutable_X(2, 0) = start_pos;
    column_plant_->mutable_X(3, 0) = 0.0;

    turret_encoder_.InitializeHallEffectAndPosition(
        column_plant_->X(2, 0) - column_plant_->X(0, 0),
        constants::GetValues().column.turret_zeroing.lower_hall_position,
        constants::GetValues().column.turret_zeroing.upper_hall_position);
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

    hood_encoder_.GetSensorValues(&position->hood);
    intake_pot_encoder_.GetSensorValues(&position->intake);
    position->theta_shooter = shooter_plant_->Y(0, 0);

    turret_encoder_.GetSensorValues(&position->column.turret);
    indexer_encoder_.GetSensorValues(&position->column.indexer);

    position.Send();
  }

  double hood_position() const { return hood_plant_->X(0, 0); }
  double hood_angular_velocity() const { return hood_plant_->X(1, 0); }

  double turret_position() const { return column_plant_->X(2, 0); }
  double turret_angular_velocity() const { return column_plant_->X(3, 0); }

  double intake_position() const { return intake_plant_->X(0, 0); }
  double intake_velocity() const { return intake_plant_->X(1, 0); }

  double shooter_velocity() const { return shooter_plant_->X(2, 0); }

  double indexer_velocity() const { return column_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_hood_voltage_offset(double voltage_offset) {
    hood_plant_->set_voltage_offset(voltage_offset);
  }

  void set_intake_voltage_offset(double voltage_offset) {
    intake_plant_->set_voltage_offset(voltage_offset);
  }

  void set_shooter_voltage_offset(double voltage_offset) {
    shooter_plant_->set_voltage_offset(voltage_offset);
  }

  void set_indexer_voltage_offset(double voltage_offset) {
    column_plant_->set_indexer_voltage_offset(voltage_offset);
  }

  void set_turret_voltage_offset(double voltage_offset) {
    column_plant_->set_turret_voltage_offset(voltage_offset);
  }

  // Triggers the indexer to freeze in position.
  void set_freeze_indexer(bool freeze_indexer) {
    freeze_indexer_ = freeze_indexer;
  }
  // Triggers the turret to freeze relative to the indexer.
  void set_freeze_turret(bool freeze_turret) {
    freeze_turret_ = freeze_turret;
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

    const double voltage_check_indexer =
        (static_cast<column::Column::State>(
             superstructure_queue_.status->turret.state) ==
         column::Column::State::RUNNING)
            ? superstructure::column::Column::kOperatingVoltage
            : superstructure::column::Column::kZeroingVoltage;

    const double voltage_check_turret =
        (static_cast<column::Column::State>(
             superstructure_queue_.status->turret.state) ==
         column::Column::State::RUNNING)
            ? superstructure::column::Column::kOperatingVoltage
            : superstructure::column::Column::kZeroingVoltage;

    const double voltage_check_intake =
        (static_cast<intake::Intake::State>(
             superstructure_queue_.status->intake.state) ==
         intake::Intake::State::RUNNING)
            ? superstructure::intake::Intake::kOperatingVoltage
            : superstructure::intake::Intake::kZeroingVoltage;

    CHECK_LE(::std::abs(superstructure_queue_.output->voltage_hood),
             voltage_check_hood);

    CHECK_LE(::std::abs(superstructure_queue_.output->voltage_intake),
             voltage_check_intake);

    EXPECT_LE(::std::abs(superstructure_queue_.output->voltage_indexer),
              voltage_check_indexer)
        << ": check voltage " << voltage_check_indexer;

    CHECK_LE(::std::abs(superstructure_queue_.output->voltage_turret),
             voltage_check_turret);

    ::Eigen::Matrix<double, 1, 1> hood_U;
    hood_U << superstructure_queue_.output->voltage_hood +
                  hood_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> intake_U;
    intake_U << superstructure_queue_.output->voltage_intake +
                    intake_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> shooter_U;
    shooter_U << superstructure_queue_.output->voltage_shooter +
                     shooter_plant_->voltage_offset();

    ::Eigen::Matrix<double, 2, 1> column_U;
    column_U << superstructure_queue_.output->voltage_indexer +
                    column_plant_->indexer_voltage_offset(),
        superstructure_queue_.output->voltage_turret +
            column_plant_->turret_voltage_offset();

    hood_plant_->Update(hood_U);
    intake_plant_->Update(intake_U);
    shooter_plant_->Update(shooter_U);

    // Freeze the indexer by setting it's voltage to 0, and then freezing the
    // position and setting the velocity to 0.
    const double indexer_position = column_plant_->X(0, 0);
    const double turret_encoder_position =
        column_plant_->X(2, 0) - column_plant_->X(0, 0);
    if (freeze_indexer_) {
      column_U(0, 0) = 0.0;
    }
    if (freeze_turret_) {
      column_U(1, 0) = 0.0;
    }

    column_plant_->Update(column_U);

    if (freeze_indexer_) {
      column_plant_->mutable_X(0, 0) = indexer_position;
      column_plant_->mutable_X(1, 0) = 0.0;
      column_plant_->UpdateY(column_U);
    }
    if (freeze_turret_) {
      column_plant_->mutable_X(2, 0) =
          column_plant_->X(0, 0) + turret_encoder_position;
      column_plant_->mutable_X(3, 0) = column_plant_->X(1, 0);
      column_plant_->UpdateY(column_U);
    }

    double angle_hood = hood_plant_->Y(0, 0);
    const double position_intake = intake_plant_->Y(0, 0);

    // The hood is special.  We don't want to fault when we hit the hard stop.
    // We want to freeze the hood at the hard stop.
    if (angle_hood > constants::Values::kHoodRange.upper_hard) {
      LOG(INFO, "At the hood upper hard stop of %f\n",
          constants::Values::kHoodRange.upper_hard);
      angle_hood = constants::Values::kHoodRange.upper_hard;
      hood_plant_->mutable_X(0, 0) = angle_hood;
      hood_plant_->mutable_X(1, 0) = 0.0;
      hood_plant_->UpdateY(hood_U);
    } else if (angle_hood < constants::Values::kHoodRange.lower_hard) {
      LOG(INFO, "At the hood lower hard stop of %f\n",
          constants::Values::kHoodRange.lower_hard);
      angle_hood = constants::Values::kHoodRange.lower_hard;
      hood_plant_->mutable_X(0, 0) = angle_hood;
      hood_plant_->mutable_X(1, 0) = 0.0;
      hood_plant_->UpdateY(hood_U);
    }

    const double angle_indexer = column_plant_->Y(0, 0);
    const double encoder_angle_turret = column_plant_->Y(1, 0);
    double angle_turret = column_plant_->X(2, 0);

    // The expected zeroing procedure involves yanking on the wires for the
    // turret in some cases.  So, implement the hard stop like the hood.
    if (angle_turret > constants::Values::kTurretRange.upper_hard) {
      LOG(INFO, "At the turret upper hard stop of %f\n",
          constants::Values::kTurretRange.upper_hard);
      angle_turret = constants::Values::kTurretRange.upper_hard;
      column_plant_->mutable_X(2, 0) = angle_turret;
      column_plant_->mutable_X(3, 0) = 0.0;

      column_plant_->UpdateY(column_U);
    } else if (angle_turret < constants::Values::kTurretRange.lower_hard) {
      LOG(INFO, "At the turret lower hard stop of %f\n",
          constants::Values::kTurretRange.lower_hard);
      angle_turret = constants::Values::kTurretRange.lower_hard;
      column_plant_->mutable_X(2, 0) = angle_turret;
      column_plant_->mutable_X(3, 0) = 0.0;

      column_plant_->UpdateY(column_U);
    }

    hood_encoder_.MoveTo(angle_hood);
    intake_pot_encoder_.MoveTo(position_intake);

    indexer_encoder_.MoveTo(angle_indexer);
    turret_encoder_.MoveTo(encoder_angle_turret);

    EXPECT_GE(angle_turret, constants::Values::kTurretRange.lower_hard);
    EXPECT_LE(angle_turret, constants::Values::kTurretRange.upper_hard);
    EXPECT_GE(position_intake, constants::Values::kIntakeRange.lower_hard);
    EXPECT_LE(position_intake, constants::Values::kIntakeRange.upper_hard);
  }

 private:
  ::std::unique_ptr<HoodPlant> hood_plant_;
  PositionSensorSimulator hood_encoder_;

  ::std::unique_ptr<IntakePlant> intake_plant_;
  PositionSensorSimulator intake_pot_encoder_;

  ::std::unique_ptr<ShooterPlant> shooter_plant_;

  PositionSensorSimulator indexer_encoder_;
  PositionSensorSimulator turret_encoder_;
  ::std::unique_ptr<ColumnPlant> column_plant_;

  bool freeze_indexer_ = false;
  bool freeze_turret_ = false;

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

    // Check that the angular velocity, average angular velocity, and estimated
    // angular velocity match when we are done for the shooter.
    EXPECT_NEAR(superstructure_queue_.goal->shooter.angular_velocity,
                superstructure_queue_.status->shooter.angular_velocity, 0.1);
    EXPECT_NEAR(superstructure_queue_.goal->shooter.angular_velocity,
                superstructure_queue_.status->shooter.avg_angular_velocity,
                0.1);
    EXPECT_NEAR(superstructure_queue_.goal->shooter.angular_velocity,
                superstructure_plant_.shooter_velocity(), 0.1);

    // Check that the angular velocity, average angular velocity, and estimated
    // angular velocity match when we are done for the indexer.
    EXPECT_NEAR(superstructure_queue_.goal->indexer.angular_velocity,
                superstructure_queue_.status->indexer.angular_velocity, 0.1);
    EXPECT_NEAR(superstructure_queue_.goal->indexer.angular_velocity,
                superstructure_queue_.status->indexer.avg_angular_velocity,
                0.1);
    EXPECT_NEAR(superstructure_queue_.goal->indexer.angular_velocity,
                superstructure_plant_.indexer_velocity(), 0.1);
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
      const auto loop_start_time = monotonic_clock::now();
      double begin_intake_velocity = superstructure_plant_.intake_velocity();
      double begin_turret_velocity =
          superstructure_plant_.turret_angular_velocity();
      double begin_hood_velocity =
          superstructure_plant_.hood_angular_velocity();

      RunIteration(enabled);

      const double loop_time =
          chrono::duration_cast<chrono::duration<double>>(
              monotonic_clock::now() - loop_start_time).count();
      const double intake_acceleration =
          (superstructure_plant_.intake_velocity() - begin_intake_velocity) /
          loop_time;
      const double turret_acceleration =
          (superstructure_plant_.turret_angular_velocity() -
           begin_turret_velocity) /
          loop_time;
      const double hood_acceleration =
          (superstructure_plant_.hood_angular_velocity() -
           begin_hood_velocity) /
          loop_time;
      EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
      EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);
      EXPECT_GE(peak_turret_acceleration_, turret_acceleration);
      EXPECT_LE(-peak_turret_acceleration_, turret_acceleration);
      EXPECT_GE(peak_hood_acceleration_, hood_acceleration);
      EXPECT_LE(-peak_hood_acceleration_, hood_acceleration);

      EXPECT_GE(peak_intake_velocity_, superstructure_plant_.intake_velocity());
      EXPECT_LE(-peak_intake_velocity_,
                superstructure_plant_.intake_velocity());
      EXPECT_GE(peak_turret_velocity_,
                superstructure_plant_.turret_angular_velocity());
      EXPECT_LE(-peak_turret_velocity_,
                superstructure_plant_.turret_angular_velocity());
      EXPECT_GE(peak_hood_velocity_,
                superstructure_plant_.hood_angular_velocity());
      EXPECT_LE(-peak_hood_velocity_,
                superstructure_plant_.hood_angular_velocity());
    }
  }

  void set_peak_intake_acceleration(double value) {
    peak_intake_acceleration_ = value;
  }
  void set_peak_turret_acceleration(double value) {
    peak_turret_acceleration_ = value;
  }
  void set_peak_hood_acceleration(double value) {
    peak_hood_acceleration_ = value;
  }
  void set_peak_intake_velocity(double value) { peak_intake_velocity_ = value; }
  void set_peak_turret_velocity(double value) { peak_turret_velocity_ = value; }
  void set_peak_hood_velocity(double value) { peak_hood_velocity_ = value; }

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory
  // that is no longer valid.
  SuperstructureQueue superstructure_queue_;

  // Create a control loop and simulation.
  Superstructure superstructure_;
  SuperstructureSimulation superstructure_plant_;

 private:
  // The acceleration limits to check for while moving.
  double peak_intake_acceleration_ = 1e10;
  double peak_turret_acceleration_ = 1e10;
  double peak_hood_acceleration_ = 1e10;
  // The velocity limits to check for while moving.
  double peak_intake_velocity_ = 1e10;
  double peak_turret_velocity_ = 1e10;
  double peak_hood_velocity_ = 1e10;
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

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
//
// We are going to disable collision detection to make this easier to implement.
TEST_F(SuperstructureTest, SaturationTest) {
  // Zero it before we move.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.distance = constants::Values::kIntakeRange.upper;
    goal->turret.angle = constants::Values::kTurretRange.upper;
    goal->hood.angle = constants::Values::kHoodRange.upper;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(8));
  VerifyNearGoal();

  superstructure_.set_ignore_collisions(true);

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.distance = constants::Values::kIntakeRange.lower;
    goal->intake.profile_params.max_velocity = 20.0;
    goal->intake.profile_params.max_acceleration = 0.1;
    goal->turret.angle = constants::Values::kTurretRange.lower;
    goal->turret.profile_params.max_velocity = 20.0;
    goal->turret.profile_params.max_acceleration = 1.0;
    goal->hood.angle = constants::Values::kHoodRange.lower;
    goal->hood.profile_params.max_velocity = 20.0;
    goal->hood.profile_params.max_acceleration = 1.0;
    ASSERT_TRUE(goal.Send());
  }
  set_peak_intake_velocity(23.0);
  set_peak_turret_velocity(23.0);
  set_peak_hood_velocity(23.0);
  set_peak_intake_acceleration(0.2);
  set_peak_turret_acceleration(1.1);
  set_peak_hood_acceleration(1.1);

  RunForTime(chrono::seconds(8));
  VerifyNearGoal();

  // Now do a high acceleration move with a low velocity limit.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->intake.distance = constants::Values::kIntakeRange.upper;
    goal->intake.profile_params.max_velocity = 0.1;
    goal->intake.profile_params.max_acceleration = 100;
    goal->turret.angle = constants::Values::kTurretRange.upper;
    goal->turret.profile_params.max_velocity = 1;
    goal->turret.profile_params.max_acceleration = 100;
    goal->hood.angle = constants::Values::kHoodRange.upper;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 100;
    ASSERT_TRUE(goal.Send());
  }

  set_peak_intake_velocity(0.2);
  set_peak_turret_velocity(1.1);
  set_peak_hood_velocity(1.1);
  set_peak_intake_acceleration(103);
  set_peak_turret_acceleration(103);
  set_peak_hood_acceleration(103);
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

  EXPECT_NEAR(column::Column::kIntakeZeroingMinDistance,
              superstructure_queue_.status->intake.position, 0.001);

  // Now, center the turret so we can try ridiculous things without having the
  // intake pushed out.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = -100.0;
    goal->hood.profile_params.max_velocity = 1;
    goal->hood.profile_params.max_acceleration = 0.5;

    goal->turret.angle = 0.0;
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

  EXPECT_NEAR(0.0, superstructure_queue_.status->turret.position, 0.001);

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

    goal->intake.distance = constants::Values::kIntakeRange.upper;
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
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());
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
    goal->intake.distance = constants::Values::kIntakeRange.upper;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

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
  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.upper);

  superstructure_plant_.InitializeTurretPosition(0.0);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.upper - 0.1;
    goal->turret.angle = constants::Values::kTurretRange.upper - 0.1;
    goal->intake.distance = constants::Values::kIntakeRange.upper - 0.1;
    goal->indexer.angular_velocity = -5.0;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(10));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());

  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));

  EXPECT_EQ(hood::Hood::State::ZEROING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::UNINITIALIZED,
            superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::ZEROING_UNINITIALIZED,
            superstructure_.column().state());

  RunForTime(chrono::seconds(10));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());
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
  EXPECT_EQ(0.0, superstructure_.intake().goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.column().goal(2, 0));

  // Now make sure they move correctly
  RunForTime(chrono::seconds(4), true);
  EXPECT_NE(0.0, superstructure_.hood().goal(0, 0));
  EXPECT_NE(0.0, superstructure_.intake().goal(0, 0));
  EXPECT_NE(0.0, superstructure_.column().goal(2, 0));
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
    goal->turret.angle = 0.0;
    goal->intake.distance = constants::Values::kIntakeRange.lower;
    ASSERT_TRUE(goal.Send());
  }

  // Run disabled for 2 seconds
  RunForTime(chrono::seconds(2), false);
  EXPECT_EQ(hood::Hood::State::DISABLED_INITIALIZED,
            superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::ZEROING_UNINITIALIZED,
            superstructure_.column().state());

  superstructure_plant_.set_hood_voltage_offset(2.0);

  superstructure_plant_.set_turret_voltage_offset(-1.5);
  superstructure_plant_.set_indexer_voltage_offset(2.0);

  RunForTime(chrono::seconds(1), false);
  superstructure_plant_.set_hood_voltage_offset(0.0);
  RunForTime(chrono::seconds(5), false);


  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());
  superstructure_plant_.set_turret_voltage_offset(0.0);
  superstructure_plant_.set_indexer_voltage_offset(0.0);

  RunForTime(chrono::seconds(4), true);

  VerifyNearGoal();
}

// TODO(austin): Test saturation

// Tests that the shooter spins up to speed and that it then spins down
// without applying any power.
TEST_F(SuperstructureTest, ShooterSpinUpAndDown) {
  // Spin up.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower + 0.03;
    goal->turret.angle = constants::Values::kTurretRange.lower + 0.03;
    goal->intake.distance = constants::Values::kIntakeRange.upper - 0.03;
    goal->shooter.angular_velocity = 300.0;
    goal->indexer.angular_velocity = 20.0;
    ASSERT_TRUE(goal.Send());
  }


  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
  EXPECT_TRUE(superstructure_queue_.status->shooter.ready);
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower + 0.03;
    goal->turret.angle = constants::Values::kTurretRange.lower + 0.03;
    goal->intake.distance = constants::Values::kIntakeRange.upper - 0.03;
    goal->shooter.angular_velocity = 0.0;
    goal->indexer.angular_velocity = 0.0;
    ASSERT_TRUE(goal.Send());
  }

  // Make sure we don't apply voltage on spin-down.
  RunIteration();
  EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
  EXPECT_EQ(0.0, superstructure_queue_.output->voltage_shooter);
  // Continue to stop.
  RunForTime(chrono::seconds(5));
  EXPECT_TRUE(superstructure_queue_.output.FetchLatest());
  EXPECT_EQ(0.0, superstructure_queue_.output->voltage_shooter);
}

// Tests that the shooter can spin up nicely after being disabled for a while.
TEST_F(SuperstructureTest, ShooterDisabled) {
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower + 0.03;
    goal->turret.angle = constants::Values::kTurretRange.lower + 0.03;
    goal->intake.distance = constants::Values::kIntakeRange.upper - 0.03;
    goal->shooter.angular_velocity = 200.0;
    goal->indexer.angular_velocity = 20.0;
    ASSERT_TRUE(goal.Send());
  }
  RunForTime(chrono::seconds(5), false);
  EXPECT_EQ(nullptr, superstructure_queue_.output.get());

  RunForTime(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that when the indexer gets stuck, it detects it and unjams.
TEST_F(SuperstructureTest, StuckIndexerTest) {
  // Spin up.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower + 0.03;
    goal->turret.angle = constants::Values::kTurretRange.lower + 0.03;
    goal->intake.distance = constants::Values::kIntakeRange.upper - 0.03;
    goal->shooter.angular_velocity = 0.0;
    goal->indexer.angular_velocity = 5.0;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
  EXPECT_TRUE(superstructure_queue_.status->indexer.ready);

  // Now, stick it.
  const auto stuck_start_time = monotonic_clock::now();
  superstructure_plant_.set_freeze_indexer(true);
  while (monotonic_clock::now() < stuck_start_time + chrono::seconds(1)) {
    RunIteration();
    superstructure_queue_.status.FetchLatest();
    ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_queue_.status->indexer.state) ==
        column::Column::IndexerState::REVERSING) {
      break;
    }
  }

  // Make sure it detected it reasonably fast.
  const auto stuck_detection_time = monotonic_clock::now();
  EXPECT_TRUE(stuck_detection_time - stuck_start_time <
              chrono::milliseconds(200));

  // Grab the position we were stuck at.
  superstructure_queue_.position.FetchLatest();
  ASSERT_TRUE(superstructure_queue_.position.get() != nullptr);
  const double indexer_position =
      superstructure_queue_.position->column.indexer.position;

  // Now, unstick it.
  superstructure_plant_.set_freeze_indexer(false);
  const auto unstuck_start_time = monotonic_clock::now();
  while (monotonic_clock::now() < unstuck_start_time + chrono::seconds(1)) {
    RunIteration();
    superstructure_queue_.status.FetchLatest();
    ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_queue_.status->indexer.state) ==
        column::Column::IndexerState::RUNNING) {
      break;
    }
  }

  // Make sure it took some time, but not too much to detect us not being stuck anymore.
  const auto unstuck_detection_time = monotonic_clock::now();
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time <
              chrono::milliseconds(200));
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time >
              chrono::milliseconds(40));

  // Verify that it actually moved.
  superstructure_queue_.position.FetchLatest();
  ASSERT_TRUE(superstructure_queue_.position.get() != nullptr);
  const double unstuck_indexer_position =
      superstructure_queue_.position->column.indexer.position;
  EXPECT_LT(unstuck_indexer_position, indexer_position - 0.1);

  // Now, verify that everything works as expected.
  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that when the indexer gets stuck forever, it switches back and forth at
// a reasonable rate.
TEST_F(SuperstructureTest, ReallyStuckIndexerTest) {
  // Spin up.
  {
    auto goal = superstructure_queue_.goal.MakeMessage();
    goal->hood.angle = constants::Values::kHoodRange.lower + 0.03;
    goal->turret.angle = constants::Values::kTurretRange.lower + 0.03;
    goal->intake.distance = constants::Values::kIntakeRange.upper - 0.03;
    goal->shooter.angular_velocity = 0.0;
    goal->indexer.angular_velocity = 5.0;
    ASSERT_TRUE(goal.Send());
  }

  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
  EXPECT_TRUE(superstructure_queue_.status->indexer.ready);

  // Now, stick it.
  const auto stuck_start_time = monotonic_clock::now();
  superstructure_plant_.set_freeze_indexer(true);
  while (monotonic_clock::now() < stuck_start_time + chrono::seconds(1)) {
    RunIteration();
    superstructure_queue_.status.FetchLatest();
    ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_queue_.status->indexer.state) ==
        column::Column::IndexerState::REVERSING) {
      break;
    }
  }

  // Make sure it detected it reasonably fast.
  const auto stuck_detection_time = monotonic_clock::now();
  EXPECT_TRUE(stuck_detection_time - stuck_start_time <
              chrono::milliseconds(200));

  // Now, try to unstick it.
  const auto unstuck_start_time = monotonic_clock::now();
  while (monotonic_clock::now() < unstuck_start_time + chrono::seconds(1)) {
    RunIteration();
    superstructure_queue_.status.FetchLatest();
    ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_queue_.status->indexer.state) ==
        column::Column::IndexerState::RUNNING) {
      break;
    }
  }

  // Make sure it took some time, but not too much to detect us not being stuck
  // anymore.
  const auto unstuck_detection_time = monotonic_clock::now();
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time <
              chrono::milliseconds(600));
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time >
              chrono::milliseconds(400));
  LOG(INFO, "Unstuck time is %ldms",
      (unstuck_detection_time - unstuck_start_time).count() / 1000000);

  // Now, make sure it transitions to stuck again after a delay.
  const auto restuck_start_time = monotonic_clock::now();
  superstructure_plant_.set_freeze_indexer(true);
  while (monotonic_clock::now() < restuck_start_time + chrono::seconds(1)) {
    RunIteration();
    superstructure_queue_.status.FetchLatest();
    ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_queue_.status->indexer.state) ==
        column::Column::IndexerState::REVERSING) {
      break;
    }
  }

  // Make sure it detected it reasonably fast.
  const auto restuck_detection_time = monotonic_clock::now();
  EXPECT_TRUE(restuck_detection_time - restuck_start_time >
              chrono::milliseconds(400));
  EXPECT_TRUE(restuck_detection_time - restuck_start_time <
              chrono::milliseconds(600));
}

// TODO(austin): What happens if either fuse blows?
// TODO(austin): What happens if either fuse blows (while zeroing or operating)?
// TODO(austin): What happens if the indexer stalls while running?
//  Aren tried it and nothing really happens.

// TODO(austin): Indexer zeroing error detection.
// TODO(austin): Detect detached turret encoder

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
