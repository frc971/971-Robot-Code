#include "y2017/control_loops/superstructure/superstructure.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
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
  SuperstructureSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")),
        hood_plant_(new HoodPlant(
            ::y2017::control_loops::superstructure::hood::MakeHoodPlant())),
        hood_encoder_(constants::Values::kHoodEncoderIndexDifference),
        intake_plant_(new IntakePlant(
            ::y2017::control_loops::superstructure::intake::MakeIntakePlant())),
        intake_pot_encoder_(constants::Values::kIntakeEncoderIndexDifference),

        shooter_plant_(new ShooterPlant(::y2017::control_loops::superstructure::
                                            shooter::MakeShooterPlant())),
        indexer_encoder_(2.0 * M_PI),
        turret_encoder_(2.0 * M_PI),
        column_plant_(new ColumnPlant(::y2017::control_loops::superstructure::
                                          column::MakeColumnPlant())) {
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

    event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            Simulate();
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
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
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::IndexPosition::Builder hood_builder =
        builder.MakeBuilder<frc971::IndexPosition>();
    flatbuffers::Offset<frc971::IndexPosition> hood_offset =
        hood_encoder_.GetSensorValues(&hood_builder);

    frc971::PotAndAbsolutePosition::Builder intake_builder =
        builder.MakeBuilder<frc971::PotAndAbsolutePosition>();
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> intake_offset =
        intake_pot_encoder_.GetSensorValues(&intake_builder);

    frc971::HallEffectAndPosition::Builder turret_builder =
        builder.MakeBuilder<frc971::HallEffectAndPosition>();
    flatbuffers::Offset<frc971::HallEffectAndPosition> turret_offset =
        turret_encoder_.GetSensorValues(&turret_builder);

    frc971::HallEffectAndPosition::Builder indexer_builder =
        builder.MakeBuilder<frc971::HallEffectAndPosition>();
    flatbuffers::Offset<frc971::HallEffectAndPosition> indexer_offset =
        indexer_encoder_.GetSensorValues(&indexer_builder);

    ColumnPosition::Builder column_builder =
        builder.MakeBuilder<ColumnPosition>();
    column_builder.add_indexer(indexer_offset);
    column_builder.add_turret(turret_offset);
    flatbuffers::Offset<ColumnPosition> column_offset = column_builder.Finish();

    Position::Builder position_builder = builder.MakeBuilder<Position>();
    position_builder.add_theta_shooter(shooter_plant_->Y(0, 0));
    position_builder.add_column(column_offset);
    position_builder.add_intake(intake_offset);
    position_builder.add_hood(hood_offset);

    builder.Send(position_builder.Finish());
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
  void set_freeze_turret(bool freeze_turret) { freeze_turret_ = freeze_turret; }

  // Simulates the superstructure for a single timestep.
  void Simulate() {
    const double last_intake_velocity = intake_velocity();
    const double last_turret_velocity = turret_angular_velocity();
    const double last_hood_velocity = hood_angular_velocity();

    EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
    EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

    const double voltage_check_hood =
        (static_cast<hood::Hood::State>(
             superstructure_status_fetcher_->hood()->state()) ==
         hood::Hood::State::RUNNING)
            ? superstructure::hood::Hood::kOperatingVoltage
            : superstructure::hood::Hood::kZeroingVoltage;

    const double voltage_check_indexer =
        (static_cast<column::Column::State>(
             superstructure_status_fetcher_->turret()->state()) ==
         column::Column::State::RUNNING)
            ? superstructure::column::Column::kOperatingVoltage
            : superstructure::column::Column::kZeroingVoltage;

    const double voltage_check_turret =
        (static_cast<column::Column::State>(
             superstructure_status_fetcher_->turret()->state()) ==
         column::Column::State::RUNNING)
            ? superstructure::column::Column::kOperatingVoltage
            : superstructure::column::Column::kZeroingVoltage;

    const double voltage_check_intake =
        (static_cast<intake::Intake::State>(
             superstructure_status_fetcher_->intake()->state()) ==
         intake::Intake::State::RUNNING)
            ? superstructure::intake::Intake::kOperatingVoltage
            : superstructure::intake::Intake::kZeroingVoltage;

    AOS_CHECK_LE(::std::abs(superstructure_output_fetcher_->voltage_hood()),
                 voltage_check_hood);

    AOS_CHECK_LE(::std::abs(superstructure_output_fetcher_->voltage_intake()),
                 voltage_check_intake);

    EXPECT_LE(::std::abs(superstructure_output_fetcher_->voltage_indexer()),
              voltage_check_indexer)
        << ": check voltage " << voltage_check_indexer;

    AOS_CHECK_LE(::std::abs(superstructure_output_fetcher_->voltage_turret()),
                 voltage_check_turret);

    ::Eigen::Matrix<double, 1, 1> hood_U;
    hood_U << superstructure_output_fetcher_->voltage_hood() +
                  hood_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> intake_U;
    intake_U << superstructure_output_fetcher_->voltage_intake() +
                    intake_plant_->voltage_offset();

    ::Eigen::Matrix<double, 1, 1> shooter_U;
    shooter_U << superstructure_output_fetcher_->voltage_shooter() +
                     shooter_plant_->voltage_offset();

    ::Eigen::Matrix<double, 2, 1> column_U;
    column_U << superstructure_output_fetcher_->voltage_indexer() +
                    column_plant_->indexer_voltage_offset(),
        superstructure_output_fetcher_->voltage_turret() +
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
      AOS_LOG(INFO, "At the hood upper hard stop of %f\n",
              constants::Values::kHoodRange.upper_hard);
      angle_hood = constants::Values::kHoodRange.upper_hard;
      hood_plant_->mutable_X(0, 0) = angle_hood;
      hood_plant_->mutable_X(1, 0) = 0.0;
      hood_plant_->UpdateY(hood_U);
    } else if (angle_hood < constants::Values::kHoodRange.lower_hard) {
      AOS_LOG(INFO, "At the hood lower hard stop of %f\n",
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
      AOS_LOG(INFO, "At the turret upper hard stop of %f\n",
              constants::Values::kTurretRange.upper_hard);
      angle_turret = constants::Values::kTurretRange.upper_hard;
      column_plant_->mutable_X(2, 0) = angle_turret;
      column_plant_->mutable_X(3, 0) = 0.0;

      column_plant_->UpdateY(column_U);
    } else if (angle_turret < constants::Values::kTurretRange.lower_hard) {
      AOS_LOG(INFO, "At the turret lower hard stop of %f\n",
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

    // Check to make sure no constriants are violated.
    const double loop_time = ::aos::time::DurationInSeconds(dt_);
    const double intake_acceleration =
        (intake_velocity() - last_intake_velocity) / loop_time;
    const double turret_acceleration =
        (turret_angular_velocity() - last_turret_velocity) / loop_time;
    const double hood_acceleration =
        (hood_angular_velocity() - last_hood_velocity) / loop_time;
    EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
    EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);
    EXPECT_GE(peak_turret_acceleration_, turret_acceleration);
    EXPECT_LE(-peak_turret_acceleration_, turret_acceleration);
    EXPECT_GE(peak_hood_acceleration_, hood_acceleration);
    EXPECT_LE(-peak_hood_acceleration_, hood_acceleration);

    EXPECT_GE(peak_intake_velocity_, intake_velocity());
    EXPECT_LE(-peak_intake_velocity_, intake_velocity());
    EXPECT_GE(peak_turret_velocity_, turret_angular_velocity());
    EXPECT_LE(-peak_turret_velocity_, turret_angular_velocity());
    EXPECT_GE(peak_hood_velocity_, hood_angular_velocity());
    EXPECT_LE(-peak_hood_velocity_, hood_angular_velocity());
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

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

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

  bool first_ = true;

  // The acceleration limits to check for while moving.
  double peak_intake_acceleration_ = 1e10;
  double peak_turret_acceleration_ = 1e10;
  double peak_hood_acceleration_ = 1e10;
  // The velocity limits to check for while moving.
  double peak_intake_velocity_ = 1e10;
  double peak_turret_velocity_ = 1e10;
  double peak_hood_velocity_ = 1e10;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : ::aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2017/config.json"),
            chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop("test")),
        superstructure_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/superstructure")),
        superstructure_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/superstructure")),
        superstructure_status_fetcher_(
            test_event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            test_event_loop_->MakeFetcher<Output>("/superstructure")),
        superstructure_position_fetcher_(
            test_event_loop_->MakeFetcher<Position>("/superstructure")),
        superstructure_event_loop_(MakeEventLoop("superstructure")),
        superstructure_(superstructure_event_loop_.get()),
        superstructure_plant_event_loop_(MakeEventLoop("plant")),
        superstructure_plant_(superstructure_plant_event_loop_.get(), dt()) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
  }

  void VerifyNearGoal() {
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();

    ASSERT_TRUE(superstructure_goal_fetcher_.get() != nullptr);
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);

    EXPECT_NEAR(superstructure_goal_fetcher_->hood()->angle(),
                superstructure_status_fetcher_->hood()->position(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->hood()->angle(),
                superstructure_plant_.hood_position(), 0.001);

    EXPECT_NEAR(superstructure_goal_fetcher_->turret()->angle(),
                superstructure_status_fetcher_->turret()->position(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->turret()->angle(),
                superstructure_plant_.turret_position(), 0.001);

    EXPECT_NEAR(superstructure_goal_fetcher_->intake()->distance(),
                superstructure_status_fetcher_->intake()->position(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->intake()->distance(),
                superstructure_plant_.intake_position(), 0.001);

    // Check that the angular velocity, average angular velocity, and estimated
    // angular velocity match when we are done for the shooter.
    EXPECT_NEAR(superstructure_goal_fetcher_->shooter()->angular_velocity(),
                superstructure_status_fetcher_->shooter()->angular_velocity(), 0.1);
    EXPECT_NEAR(superstructure_goal_fetcher_->shooter()->angular_velocity(),
                superstructure_status_fetcher_->shooter()->avg_angular_velocity(),
                0.1);
    EXPECT_NEAR(superstructure_goal_fetcher_->shooter()->angular_velocity(),
                superstructure_plant_.shooter_velocity(), 0.1);

    // Check that the angular velocity, average angular velocity, and estimated
    // angular velocity match when we are done for the indexer.
    EXPECT_NEAR(superstructure_goal_fetcher_->indexer()->angular_velocity(),
                superstructure_status_fetcher_->indexer()->angular_velocity(), 0.1);
    EXPECT_NEAR(superstructure_goal_fetcher_->indexer()->angular_velocity(),
                superstructure_status_fetcher_->indexer()->avg_angular_velocity(),
                0.1);
    EXPECT_NEAR(superstructure_goal_fetcher_->indexer()->angular_velocity(),
                superstructure_plant_.indexer_velocity(), 0.1);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;

  // Create a control loop and simulation.
  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop_;
  Superstructure superstructure_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;
};

// Tests that the superstructure does nothing when the goal is zero.
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(0.2);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(0.0);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(0.05);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(5));

  VerifyNearGoal();

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
}

// Tests that the hood, turret and intake loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);

  // Set a reasonable goal.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(0.1);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(0.1);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(0.1);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
//
// We are going to disable collision detection to make this easier to implement.
TEST_F(SuperstructureTest, SaturationTest) {
  SetEnabled(true);

  // Zero it before we move.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.upper);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.upper);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(8));
  VerifyNearGoal();

  superstructure_.set_ignore_collisions(true);

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 20.0, 1.0);
    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 20.0, 1.0);
    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 20.0, 0.1);
    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.lower);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  superstructure_plant_.set_peak_intake_velocity(23.0);
  superstructure_plant_.set_peak_turret_velocity(23.0);
  superstructure_plant_.set_peak_hood_velocity(23.0);
  superstructure_plant_.set_peak_intake_acceleration(0.2);
  superstructure_plant_.set_peak_turret_acceleration(1.1);
  superstructure_plant_.set_peak_hood_acceleration(1.1);

  RunFor(chrono::seconds(8));
  VerifyNearGoal();

  // Now do a high acceleration move with a low velocity limit.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 100.0);
    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.upper);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 100.0);
    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.upper);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 0.1, 100.0);
    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_peak_intake_velocity(0.2);
  superstructure_plant_.set_peak_turret_velocity(1.1);
  superstructure_plant_.set_peak_hood_velocity(1.1);
  superstructure_plant_.set_peak_intake_acceleration(103);
  superstructure_plant_.set_peak_turret_acceleration(103);
  superstructure_plant_.set_peak_hood_acceleration(103);
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the hood, turret and intake loops doesn't try and go beyond the
// physical range of the mechanisms.
TEST_F(SuperstructureTest, RespectsRange) {
  SetEnabled(true);

  // Set some ridiculous goals to test upper limits.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(100.0);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(100.0);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(100.0);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(constants::Values::kHoodRange.upper,
              superstructure_status_fetcher_->hood()->position(), 0.001);

  EXPECT_NEAR(constants::Values::kTurretRange.upper,
              superstructure_status_fetcher_->turret()->position(), 0.001);

  EXPECT_NEAR(constants::Values::kIntakeRange.upper,
              superstructure_status_fetcher_->intake()->position(), 0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(-100.0);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(-100.0);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(-100.0);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(constants::Values::kHoodRange.lower,
              superstructure_status_fetcher_->hood()->position(), 0.001);

  EXPECT_NEAR(constants::Values::kTurretRange.lower,
              superstructure_status_fetcher_->turret()->position(), 0.001);

  EXPECT_NEAR(column::Column::kIntakeZeroingMinDistance,
              superstructure_status_fetcher_->intake()->position(), 0.001);

  // Now, center the turret so we can try ridiculous things without having the
  // intake pushed out.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(-100.0);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(0.0);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(-100.0);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(constants::Values::kHoodRange.lower,
              superstructure_status_fetcher_->hood()->position(), 0.001);

  EXPECT_NEAR(0.0, superstructure_status_fetcher_->turret()->position(), 0.001);

  EXPECT_NEAR(constants::Values::kIntakeRange.lower,
              superstructure_status_fetcher_->intake()->position(), 0.001);
}

// Tests that the hood, turret and intake loops zeroes when run for a while.
TEST_F(SuperstructureTest, ZeroTest) {
  SetEnabled(true);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 1.0, 0.5);
    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  RunFor(chrono::seconds(5));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());
}

TEST_F(SuperstructureTest, LowerHardstopStartup) {
  SetEnabled(true);
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.lower_hard);

  superstructure_plant_.InitializeTurretPosition(
      constants::Values::kTurretRange.lower_hard);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower_hard);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, UpperHardstopStartup) {
  SetEnabled(true);
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.upper_hard);

  superstructure_plant_.InitializeTurretPosition(
      constants::Values::kTurretRange.upper_hard);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper_hard);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.upper);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.upper);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  SetEnabled(true);
  superstructure_plant_.InitializeHoodPosition(
      constants::Values::kHoodRange.upper);

  superstructure_plant_.InitializeTurretPosition(0.0);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.upper - 0.1);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.upper - 0.1);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper - 0.1);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(-5.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());

  VerifyNearGoal();
  SimulateSensorReset();
  RunFor(chrono::milliseconds(100));

  EXPECT_EQ(hood::Hood::State::ZEROING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::UNINITIALIZED,
            superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::ZEROING_UNINITIALIZED,
            superstructure_.column().state());

  RunFor(chrono::seconds(10));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(SuperstructureTest, DisabledGoalTest) {
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower + 0.03);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower + 0.03);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.lower + 0.03);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::milliseconds(100));
  EXPECT_EQ(0.0, superstructure_.hood().goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.intake().goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.column().goal(2, 0));

  // Now make sure they move correctly
  SetEnabled(true);
  RunFor(chrono::seconds(4));
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
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(0.0);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.lower);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_shooter(shooter_offset);
    goal_builder.add_indexer(indexer_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Run disabled for 2 seconds
  RunFor(chrono::seconds(2));
  EXPECT_EQ(hood::Hood::State::DISABLED_INITIALIZED,
            superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::ZEROING_UNINITIALIZED,
            superstructure_.column().state());

  superstructure_plant_.set_hood_voltage_offset(2.0);

  superstructure_plant_.set_turret_voltage_offset(-1.0);
  superstructure_plant_.set_indexer_voltage_offset(2.0);

  RunFor(chrono::seconds(1));
  superstructure_plant_.set_hood_voltage_offset(0.0);
  RunFor(chrono::seconds(5));

  EXPECT_EQ(hood::Hood::State::RUNNING, superstructure_.hood().state());
  EXPECT_EQ(intake::Intake::State::RUNNING, superstructure_.intake().state());
  EXPECT_EQ(column::Column::State::RUNNING, superstructure_.column().state());
  superstructure_plant_.set_turret_voltage_offset(0.0);
  superstructure_plant_.set_indexer_voltage_offset(0.0);

  SetEnabled(true);
  RunFor(chrono::seconds(4));

  VerifyNearGoal();
}

// TODO(austin): Test saturation

// Tests that the shooter spins up to speed and that it then spins down
// without applying any power.
TEST_F(SuperstructureTest, ShooterSpinUpAndDown) {
  SetEnabled(true);

  // Spin up.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower + 0.03);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower + 0.03);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper - 0.03);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(300.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(20.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_indexer(indexer_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));
  VerifyNearGoal();
  EXPECT_TRUE(superstructure_status_fetcher_->shooter()->ready());
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower + 0.03);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower + 0.03);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper - 0.03);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_indexer(indexer_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Make sure we don't apply voltage on spin-down.
  RunFor(dt());

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
  EXPECT_EQ(0.0, superstructure_output_fetcher_->voltage_shooter());
  // Continue to stop.
  RunFor(chrono::seconds(5));
  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
  EXPECT_EQ(0.0, superstructure_output_fetcher_->voltage_shooter());
}

// Tests that the shooter can spin up nicely after being disabled for a while.
TEST_F(SuperstructureTest, ShooterDisabled) {
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower + 0.03);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower + 0.03);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper - 0.03);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(200.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(20.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_indexer(indexer_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(5));
  EXPECT_EQ(nullptr, superstructure_output_fetcher_.get());

  SetEnabled(true);
  RunFor(chrono::seconds(5));

  VerifyNearGoal();
}

// Tests that when the indexer gets stuck, it detects it and unjams.
TEST_F(SuperstructureTest, StuckIndexerTest) {
  SetEnabled(true);

  // Spin up.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower + 0.03);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower + 0.03);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper - 0.03);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(5.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_indexer(indexer_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));
  VerifyNearGoal();
  EXPECT_TRUE(superstructure_status_fetcher_->indexer()->ready());

  // Now, stick it.
  const auto stuck_start_time = monotonic_now();
  superstructure_plant_.set_freeze_indexer(true);
  while (monotonic_now() < stuck_start_time + chrono::seconds(1)) {
    RunFor(dt());
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_status_fetcher_->indexer()->state()) ==
        column::Column::IndexerState::REVERSING) {
      break;
    }
  }

  // Make sure it detected it reasonably fast.
  const auto stuck_detection_time = monotonic_now();
  EXPECT_TRUE(stuck_detection_time - stuck_start_time <
              chrono::milliseconds(200));

  // Grab the position we were stuck at.
  superstructure_position_fetcher_.Fetch();
  ASSERT_TRUE(superstructure_position_fetcher_.get() != nullptr);
  const double indexer_position =
      superstructure_position_fetcher_->column()->indexer()->encoder();

  // Now, unstick it.
  superstructure_plant_.set_freeze_indexer(false);
  const auto unstuck_start_time = monotonic_now();
  while (monotonic_now() < unstuck_start_time + chrono::seconds(1)) {
    RunFor(dt());
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_status_fetcher_->indexer()->state()) ==
        column::Column::IndexerState::RUNNING) {
      break;
    }
  }

  // Make sure it took some time, but not too much to detect us not being stuck
  // anymore.
  const auto unstuck_detection_time = monotonic_now();
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time <
              chrono::milliseconds(600));
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time >
              chrono::milliseconds(100));

  // Verify that it actually moved.
  superstructure_position_fetcher_.Fetch();
  ASSERT_TRUE(superstructure_position_fetcher_.get() != nullptr);
  const double unstuck_indexer_position =
      superstructure_position_fetcher_->column()->indexer()->encoder();
  EXPECT_LT(unstuck_indexer_position, indexer_position - 0.1);

  // Now, verify that everything works as expected.
  RunFor(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that when the indexer gets stuck forever, it switches back and forth at
// a reasonable rate.
TEST_F(SuperstructureTest, ReallyStuckIndexerTest) {
  SetEnabled(true);

  // Spin up.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    HoodGoal::Builder hood_builder = builder.MakeBuilder<HoodGoal>();
    hood_builder.add_angle(constants::Values::kHoodRange.lower + 0.03);
    flatbuffers::Offset<HoodGoal> hood_offset = hood_builder.Finish();

    TurretGoal::Builder turret_builder = builder.MakeBuilder<TurretGoal>();
    turret_builder.add_angle(constants::Values::kTurretRange.lower + 0.03);
    flatbuffers::Offset<TurretGoal> turret_offset = turret_builder.Finish();

    IntakeGoal::Builder intake_builder = builder.MakeBuilder<IntakeGoal>();
    intake_builder.add_distance(constants::Values::kIntakeRange.upper - 0.03);
    flatbuffers::Offset<IntakeGoal> intake_offset = intake_builder.Finish();

    ShooterGoal::Builder shooter_builder = builder.MakeBuilder<ShooterGoal>();
    shooter_builder.add_angular_velocity(0.0);
    flatbuffers::Offset<ShooterGoal> shooter_offset = shooter_builder.Finish();

    IndexerGoal::Builder indexer_builder = builder.MakeBuilder<IndexerGoal>();
    indexer_builder.add_angular_velocity(5.0);
    flatbuffers::Offset<IndexerGoal> indexer_offset = indexer_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_hood(hood_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_indexer(indexer_offset);
    goal_builder.add_shooter(shooter_offset);

    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));
  VerifyNearGoal();
  EXPECT_TRUE(superstructure_status_fetcher_->indexer()->ready());

  // Now, stick it.
  const auto stuck_start_time = monotonic_now();
  superstructure_plant_.set_freeze_indexer(true);
  while (monotonic_now() < stuck_start_time + chrono::seconds(1)) {
    RunFor(dt());
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_status_fetcher_->indexer()->state()) ==
        column::Column::IndexerState::REVERSING) {
      break;
    }
  }

  // Make sure it detected it reasonably fast.
  const auto stuck_detection_time = monotonic_now();
  EXPECT_TRUE(stuck_detection_time - stuck_start_time <
              chrono::milliseconds(200));

  // Now, try to unstick it.
  const auto unstuck_start_time = monotonic_now();
  while (monotonic_now() < unstuck_start_time + chrono::seconds(1)) {
    RunFor(dt());
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_status_fetcher_->indexer()->state()) ==
        column::Column::IndexerState::RUNNING) {
      break;
    }
  }

  // Make sure it took some time, but not too much to detect us not being stuck
  // anymore.
  const auto unstuck_detection_time = monotonic_now();
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time <
              chrono::milliseconds(1050));
  EXPECT_TRUE(unstuck_detection_time - unstuck_start_time >
              chrono::milliseconds(400));
  AOS_LOG(INFO, "Unstuck time is %" PRId64 "ms",
          static_cast<int64_t>(
              (unstuck_detection_time - unstuck_start_time).count() / 1000000));

  // Now, make sure it transitions to stuck again after a delay.
  const auto restuck_start_time = monotonic_now();
  superstructure_plant_.set_freeze_indexer(true);
  while (monotonic_now() < restuck_start_time + chrono::seconds(1)) {
    RunFor(dt());
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);
    if (static_cast<column::Column::IndexerState>(
            superstructure_status_fetcher_->indexer()->state()) ==
        column::Column::IndexerState::REVERSING) {
      break;
    }
  }

  // Make sure it detected it reasonably fast.
  const auto restuck_detection_time = monotonic_now();
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
