#include "y2016/control_loops/superstructure/superstructure.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/commonmath.h"
#include "aos/controls/control_loop_test.h"
#include "aos/time/time.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2016/control_loops/superstructure/arm_plant.h"
#include "y2016/control_loops/superstructure/intake_plant.h"
#include "y2016/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2016/control_loops/superstructure/superstructure_output_generated.h"
#include "y2016/control_loops/superstructure/superstructure_position_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"

#include "y2016/constants.h"

using ::frc971::control_loops::PositionSensorSimulator;

namespace y2016 {
namespace control_loops {
namespace superstructure {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

class ArmPlant : public StateFeedbackPlant<4, 2, 2> {
 public:
  explicit ArmPlant(StateFeedbackPlant<4, 2, 2> &&other)
      : StateFeedbackPlant<4, 2, 2>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 2, 1> &U) override {
    EXPECT_LT(U(0, 0), U_max(0, 0) + 0.00001 + shoulder_voltage_offset_);
    EXPECT_GT(U(0, 0), U_min(0, 0) - 0.00001 + shoulder_voltage_offset_);
    EXPECT_LT(U(1, 0), U_max(1, 0) + 0.00001 + wrist_voltage_offset_);
    EXPECT_GT(U(1, 0), U_min(1, 0) - 0.00001 + wrist_voltage_offset_);
  }

  double shoulder_voltage_offset() const { return shoulder_voltage_offset_; }
  void set_shoulder_voltage_offset(double shoulder_voltage_offset) {
    shoulder_voltage_offset_ = shoulder_voltage_offset;
  }

  double wrist_voltage_offset() const { return wrist_voltage_offset_; }
  void set_wrist_voltage_offset(double wrist_voltage_offset) {
    wrist_voltage_offset_ = wrist_voltage_offset;
  }

 private:
  double shoulder_voltage_offset_ = 0.0;
  double wrist_voltage_offset_ = 0.0;
};

class IntakePlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit IntakePlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 1, 1> &U) override {
    for (int i = 0; i < kNumInputs; ++i) {
      EXPECT_LE(U(i, 0), U_max(i, 0) + 0.00001 + voltage_offset_);
      EXPECT_GE(U(i, 0), U_min(i, 0) - 0.00001 + voltage_offset_);
    }
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
  static constexpr double kNoiseScalar = 0.1;
  SuperstructureSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")),
        intake_plant_(new IntakePlant(MakeIntakePlant())),
        arm_plant_(new ArmPlant(MakeArmPlant())),
        pot_encoder_intake_(constants::Values::kIntakeEncoderIndexDifference),
        pot_encoder_shoulder_(
            constants::Values::kShoulderEncoderIndexDifference),
        pot_encoder_wrist_(constants::Values::kWristEncoderIndexDifference) {
    InitializeIntakePosition(0.0);
    InitializeShoulderPosition(0.0);
    InitializeRelativeWristPosition(0.0);

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

  void InitializeIntakePosition(double start_pos) {
    intake_plant_->mutable_X(0, 0) = start_pos;
    intake_plant_->mutable_X(1, 0) = 0.0;

    pot_encoder_intake_.Initialize(start_pos, kNoiseScalar);
  }

  void InitializeShoulderPosition(double start_pos) {
    arm_plant_->mutable_X(0, 0) = start_pos;
    arm_plant_->mutable_X(1, 0) = 0.0;

    pot_encoder_shoulder_.Initialize(start_pos, kNoiseScalar);
  }

  // Must be called after any changes to InitializeShoulderPosition.
  void InitializeRelativeWristPosition(double start_pos) {
    arm_plant_->mutable_X(2, 0) = start_pos + arm_plant_->X(0, 0);
    arm_plant_->mutable_X(3, 0) = 0.0;

    pot_encoder_wrist_.Initialize(start_pos, kNoiseScalar);
  }

  // Must be called after any changes to InitializeShoulderPosition.
  void InitializeAbsoluteWristPosition(double start_pos) {
    InitializeRelativeWristPosition(start_pos - arm_plant_->X(0, 0));
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    frc971::PotAndIndexPosition::Builder intake_builder =
        builder.MakeBuilder<frc971::PotAndIndexPosition>();

    flatbuffers::Offset<frc971::PotAndIndexPosition> intake_offset =
        pot_encoder_intake_.GetSensorValues(&intake_builder);

    frc971::PotAndIndexPosition::Builder shoulder_builder =
        builder.MakeBuilder<frc971::PotAndIndexPosition>();
    flatbuffers::Offset<frc971::PotAndIndexPosition> shoulder_offset =
        pot_encoder_shoulder_.GetSensorValues(&shoulder_builder);

    frc971::PotAndIndexPosition::Builder wrist_builder =
        builder.MakeBuilder<frc971::PotAndIndexPosition>();
    flatbuffers::Offset<frc971::PotAndIndexPosition> wrist_offset =
        pot_encoder_wrist_.GetSensorValues(&wrist_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_intake(intake_offset);
    position_builder.add_shoulder(shoulder_offset);
    position_builder.add_wrist(wrist_offset);

    builder.Send(position_builder.Finish());
  }

  double shoulder_angle() const { return arm_plant_->X(0, 0); }
  double shoulder_angular_velocity() const { return arm_plant_->X(1, 0); }
  double wrist_angle() const { return arm_plant_->X(2, 0); }
  double wrist_angular_velocity() const { return arm_plant_->X(3, 0); }
  double intake_angle() const { return intake_plant_->X(0, 0); }
  double intake_angular_velocity() const { return intake_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_power_error(double power_error_intake, double power_error_shoulder,
                       double power_error_wrist) {
    intake_plant_->set_voltage_offset(power_error_intake);
    arm_plant_->set_shoulder_voltage_offset(power_error_shoulder);
    arm_plant_->set_wrist_voltage_offset(power_error_wrist);
  }

  // Simulates for a single timestep.
  void Simulate() {
    const double begin_shoulder_velocity = shoulder_angular_velocity();
    const double begin_intake_velocity = intake_angular_velocity();
    const double begin_wrist_velocity = wrist_angular_velocity();
    EXPECT_TRUE(superstructure_output_fetcher_.Fetch());

    // Feed voltages into physics simulation.
    ::Eigen::Matrix<double, 1, 1> intake_U;
    ::Eigen::Matrix<double, 2, 1> arm_U;
    intake_U << superstructure_output_fetcher_->voltage_intake() +
                    intake_plant_->voltage_offset();

    arm_U << superstructure_output_fetcher_->voltage_shoulder() +
                 arm_plant_->shoulder_voltage_offset(),
        superstructure_output_fetcher_->voltage_wrist() +
            arm_plant_->wrist_voltage_offset();

    // Verify that the correct power limits are being respected depending on
    // which mode we are in.
    EXPECT_TRUE(superstructure_status_fetcher_.Fetch());
    if (superstructure_status_fetcher_->state() == Superstructure::RUNNING ||
        superstructure_status_fetcher_->state() ==
            Superstructure::LANDING_RUNNING) {
      AOS_CHECK_LE(::std::abs(superstructure_output_fetcher_->voltage_intake()),
                   Superstructure::kOperatingVoltage);
      AOS_CHECK_LE(
          ::std::abs(superstructure_output_fetcher_->voltage_shoulder()),
          Superstructure::kOperatingVoltage);
      AOS_CHECK_LE(::std::abs(superstructure_output_fetcher_->voltage_wrist()),
                   Superstructure::kOperatingVoltage);
    } else {
      AOS_CHECK_LE(::std::abs(superstructure_output_fetcher_->voltage_intake()),
                   Superstructure::kZeroingVoltage);
      AOS_CHECK_LE(
          ::std::abs(superstructure_output_fetcher_->voltage_shoulder()),
          Superstructure::kZeroingVoltage);
      AOS_CHECK_LE(::std::abs(superstructure_output_fetcher_->voltage_wrist()),
                   Superstructure::kZeroingVoltage);
    }
    if (arm_plant_->X(0, 0) <=
        Superstructure::kShoulderTransitionToLanded + 1e-4) {
      AOS_CHECK_GE(superstructure_output_fetcher_->voltage_shoulder(),
                   Superstructure::kLandingShoulderDownVoltage - 0.00001);
    }

    // Use the plant to generate the next physical state given the voltages to
    // the motors.
    intake_plant_->Update(intake_U);

    {
      const double bemf_voltage = arm_plant_->X(1, 0) / kV_shoulder;
      bool is_accelerating = false;
      if (bemf_voltage > 0) {
        is_accelerating = arm_U(0, 0) > bemf_voltage;
      } else {
        is_accelerating = arm_U(0, 0) < bemf_voltage;
      }
      if (is_accelerating) {
        arm_plant_->set_index(0);
      } else {
        arm_plant_->set_index(1);
      }
    }
    arm_plant_->Update(arm_U);

    const double angle_intake = intake_plant_->Y(0, 0);
    const double angle_shoulder = arm_plant_->Y(0, 0);
    const double angle_wrist = arm_plant_->Y(1, 0);

    // Use the physical state to simulate sensor readings.
    pot_encoder_intake_.MoveTo(angle_intake);
    pot_encoder_shoulder_.MoveTo(angle_shoulder);
    pot_encoder_wrist_.MoveTo(angle_wrist);

    // Validate that everything is within range.
    EXPECT_GE(angle_intake, constants::Values::kIntakeRange.lower_hard);
    EXPECT_LE(angle_intake, constants::Values::kIntakeRange.upper_hard);
    EXPECT_GE(angle_shoulder, constants::Values::kShoulderRange.lower_hard);
    EXPECT_LE(angle_shoulder, constants::Values::kShoulderRange.upper_hard);
    EXPECT_GE(angle_wrist, constants::Values::kWristRange.lower_hard);
    EXPECT_LE(angle_wrist, constants::Values::kWristRange.upper_hard);

    const double loop_time = ::aos::time::DurationInSeconds(dt_);
    const double shoulder_acceleration =
        (shoulder_angular_velocity() - begin_shoulder_velocity) / loop_time;
    const double intake_acceleration =
        (intake_angular_velocity() - begin_intake_velocity) / loop_time;
    const double wrist_acceleration =
        (wrist_angular_velocity() - begin_wrist_velocity) / loop_time;
    EXPECT_GE(peak_shoulder_acceleration_, shoulder_acceleration);
    EXPECT_LE(-peak_shoulder_acceleration_, shoulder_acceleration);
    EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
    EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);
    EXPECT_GE(peak_wrist_acceleration_, wrist_acceleration);
    EXPECT_LE(-peak_wrist_acceleration_, wrist_acceleration);

    EXPECT_GE(peak_shoulder_velocity_, shoulder_angular_velocity());
    EXPECT_LE(-peak_shoulder_velocity_, shoulder_angular_velocity());
    EXPECT_GE(peak_intake_velocity_, intake_angular_velocity());
    EXPECT_LE(-peak_intake_velocity_, intake_angular_velocity());
    EXPECT_GE(peak_wrist_velocity_, wrist_angular_velocity());
    EXPECT_LE(-peak_wrist_velocity_, wrist_angular_velocity());

    if (check_for_collisions_) {
      ASSERT_FALSE(collided());
    }
  }

  // Runs iterations while watching the average acceleration per cycle and
  // making sure it doesn't exceed the provided bounds.
  void set_peak_intake_acceleration(double value) {
    peak_intake_acceleration_ = value;
  }
  void set_peak_shoulder_acceleration(double value) {
    peak_shoulder_acceleration_ = value;
  }
  void set_peak_wrist_acceleration(double value) {
    peak_wrist_acceleration_ = value;
  }
  void set_peak_intake_velocity(double value) { peak_intake_velocity_ = value; }
  void set_peak_shoulder_velocity(double value) {
    peak_shoulder_velocity_ = value;
  }
  void set_peak_wrist_velocity(double value) { peak_wrist_velocity_ = value; }

  void set_check_for_collisions(bool check_for_collisions) {
    check_for_collisions_ = check_for_collisions;
  }

  bool collided() const {
    return CollisionAvoidance::collided_with_given_angles(
        shoulder_angle(), wrist_angle(), intake_angle());
  }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;

  bool first_ = true;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  ::std::unique_ptr<IntakePlant> intake_plant_;
  ::std::unique_ptr<ArmPlant> arm_plant_;

  PositionSensorSimulator pot_encoder_intake_;
  PositionSensorSimulator pot_encoder_shoulder_;
  PositionSensorSimulator pot_encoder_wrist_;

  bool check_for_collisions_ = true;
  // The acceleration limits to check for while moving for the 3 axes.
  double peak_intake_acceleration_ = 1e10;
  double peak_shoulder_acceleration_ = 1e10;
  double peak_wrist_acceleration_ = 1e10;
  // The velocity limits to check for while moving for the 3 axes.
  double peak_intake_velocity_ = 1e10;
  double peak_shoulder_velocity_ = 1e10;
  double peak_wrist_velocity_ = 1e10;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : ::aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2016/config.json"),
            chrono::microseconds(5000)),
        test_event_loop_(MakeEventLoop("test")),
        superstructure_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/superstructure")),
        superstructure_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/superstructure")),
        superstructure_status_fetcher_(
            test_event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_event_loop_(MakeEventLoop("superstructure")),
        superstructure_(superstructure_event_loop_.get()),
        superstructure_plant_event_loop_(MakeEventLoop("plant")),
        superstructure_plant_(superstructure_plant_event_loop_.get(), dt()) {}

  void VerifyNearGoal() {
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();

    EXPECT_TRUE(superstructure_goal_fetcher_.get() != nullptr);
    EXPECT_TRUE(superstructure_status_fetcher_.get() != nullptr);

    EXPECT_NEAR(superstructure_goal_fetcher_->angle_intake(),
                superstructure_status_fetcher_->intake()->angle(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->angle_shoulder(),
                superstructure_status_fetcher_->shoulder()->angle(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->angle_wrist(),
                superstructure_status_fetcher_->wrist()->angle(), 0.001);

    EXPECT_NEAR(superstructure_goal_fetcher_->angle_intake(),
                superstructure_plant_.intake_angle(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->angle_shoulder(),
                superstructure_plant_.shoulder_angle(), 0.001);
    EXPECT_NEAR(superstructure_goal_fetcher_->angle_wrist(),
                superstructure_plant_.wrist_angle(), 0.001);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;

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
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0);
    goal_builder.add_angle_shoulder(0);
    goal_builder.add_angle_wrist(0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that the loop can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  // Set a reasonable goal.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(M_PI / 4.0);
    goal_builder.add_angle_shoulder(1.4);
    goal_builder.add_angle_wrist(M_PI / 4.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the loop doesn't try and go beyond the physical range of the
// mechanisms.
TEST_F(SuperstructureTest, RespectsRange) {
  SetEnabled(true);
  // Set some ridiculous goals to test upper limits.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(M_PI * 10);
    goal_builder.add_angle_shoulder(M_PI * 10);
    goal_builder.add_angle_wrist(M_PI * 10);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(constants::Values::kIntakeRange.upper,
              superstructure_status_fetcher_->intake()->angle(), 0.001);
  EXPECT_NEAR(constants::Values::kShoulderRange.upper,
              superstructure_status_fetcher_->shoulder()->angle(), 0.001);
  EXPECT_NEAR(constants::Values::kWristRange.upper +
                  constants::Values::kShoulderRange.upper,
              superstructure_status_fetcher_->wrist()->angle(), 0.001);

  // Set some ridiculous goals to test limits.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(M_PI * 10);
    goal_builder.add_angle_shoulder(M_PI * 10);
    goal_builder.add_angle_wrist(-M_PI * 10.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(constants::Values::kIntakeRange.upper,
              superstructure_status_fetcher_->intake()->angle(), 0.001);
  EXPECT_NEAR(constants::Values::kShoulderRange.upper,
              superstructure_status_fetcher_->shoulder()->angle(), 0.001);
  EXPECT_NEAR(constants::Values::kWristRange.lower +
                  constants::Values::kShoulderRange.upper,
              superstructure_status_fetcher_->wrist()->angle(), 0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(-M_PI * 10);
    goal_builder.add_angle_shoulder(-M_PI * 10);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_status_fetcher_.Fetch();
  EXPECT_NEAR(constants::Values::kIntakeRange.lower,
              superstructure_status_fetcher_->intake()->angle(), 0.001);
  EXPECT_NEAR(constants::Values::kShoulderRange.lower,
              superstructure_status_fetcher_->shoulder()->angle(), 0.001);
  EXPECT_NEAR(0.0, superstructure_status_fetcher_->wrist()->angle(), 0.001);
}

// Tests that the loop zeroes when run for a while.
TEST_F(SuperstructureTest, ZeroTest) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(constants::Values::kIntakeRange.lower);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.lower);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  RunFor(chrono::seconds(5));

  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
}

// Tests that starting at the lower hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, LowerHardstopStartup) {
  SetEnabled(true);
  // Don't check for collisions for this test.
  superstructure_plant_.set_check_for_collisions(false);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.lower);
  superstructure_plant_.InitializeRelativeWristPosition(
      constants::Values::kWristRange.lower);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(constants::Values::kIntakeRange.upper);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.upper);
    goal_builder.add_angle_wrist(constants::Values::kWristRange.upper +
                                 constants::Values::kShoulderRange.upper);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  // We have to wait for it to put the elevator in a safe position as well.
  RunFor(chrono::seconds(15));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, UpperHardstopStartup) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.upper);
  superstructure_plant_.InitializeRelativeWristPosition(
      constants::Values::kWristRange.upper);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(constants::Values::kIntakeRange.lower);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.lower);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  // We have to wait for it to put the superstructure in a safe position as
  // well.
  RunFor(chrono::seconds(20));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.upper);
  superstructure_plant_.InitializeRelativeWristPosition(
      constants::Values::kWristRange.upper);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(constants::Values::kIntakeRange.lower + 0.3);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.upper);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(15));

  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
  VerifyNearGoal();
  SimulateSensorReset();
  RunFor(chrono::milliseconds(100));
  EXPECT_NE(Superstructure::RUNNING, superstructure_.state());
  RunFor(chrono::milliseconds(10000));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(SuperstructureTest, DisabledGoalTest) {
  // Don't check for collisions for this test.
  superstructure_plant_.set_check_for_collisions(false);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(constants::Values::kIntakeRange.lower + 0.03);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.lower +
                                    0.03);
    goal_builder.add_angle_wrist(constants::Values::kWristRange.lower + 0.03);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::milliseconds(100));
  EXPECT_EQ(0.0, superstructure_.intake_.goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.arm_.goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.arm_.goal(2, 0));

  // Now make sure they move correctly
  SetEnabled(true);
  RunFor(chrono::milliseconds(4000));
  EXPECT_NE(0.0, superstructure_.intake_.goal(0, 0));
  EXPECT_NE(0.0, superstructure_.arm_.goal(0, 0));
  EXPECT_NE(0.0, superstructure_.arm_.goal(2, 0));
}

// Tests that disabling while zeroing at any state restarts from beginning
TEST_F(SuperstructureTest, DisabledWhileZeroingHigh) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.upper);
  superstructure_plant_.InitializeAbsoluteWristPosition(
      constants::Values::kWristRange.upper);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(constants::Values::kIntakeRange.upper);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.upper);
    goal_builder.add_angle_wrist(constants::Values::kWristRange.upper);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Expected states to cycle through and check in order.
  Superstructure::State kExpectedStateOrder[] = {
      Superstructure::DISABLED_INITIALIZED,
      Superstructure::HIGH_ARM_ZERO_LIFT_ARM,
      Superstructure::HIGH_ARM_ZERO_LEVEL_SHOOTER,
      Superstructure::HIGH_ARM_ZERO_MOVE_INTAKE_OUT,
      Superstructure::HIGH_ARM_ZERO_LOWER_ARM,
  };

  // Cycle through until arm_ and intake_ are initialized in superstructure.cc
  while (superstructure_.state() < Superstructure::DISABLED_INITIALIZED) {
    RunFor(dt());
  }

  static const int kNumberOfStates =
      sizeof(kExpectedStateOrder) / sizeof(kExpectedStateOrder[0]);

  // Next state when reached to disable
  for (int i = 0; i < kNumberOfStates; i++) {
    // Next expected state after being disabled that is expected until next
    //  state to disable at is reached
    for (int j = 0; superstructure_.state() != kExpectedStateOrder[i] && j <= i;
         j++) {
      // RunIteration until next expected state is reached with a maximum
      //  of 10000 times to ensure a breakout
      for (int o = 0;
           superstructure_.state() < kExpectedStateOrder[j] && o < 10000; o++) {
        RunFor(dt());
        EXPECT_LT(o, 9999);
      }
      EXPECT_EQ(kExpectedStateOrder[j], superstructure_.state());
    }

    EXPECT_EQ(kExpectedStateOrder[i], superstructure_.state());

    // Disable
    SetEnabled(false);
    RunFor(dt());
    SetEnabled(true);

    EXPECT_EQ(Superstructure::DISABLED_INITIALIZED, superstructure_.state());
  }

  SetEnabled(true);
  RunFor(chrono::seconds(10));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
}

// Tests that disabling while zeroing at any state restarts from beginning
TEST_F(SuperstructureTest, DisabledWhileZeroingLow) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.lower);
  superstructure_plant_.InitializeAbsoluteWristPosition(0.0);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(constants::Values::kIntakeRange.lower);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.lower);
    goal_builder.add_angle_wrist(constants::Values::kWristRange.lower);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Expected states to cycle through and check in order.
  Superstructure::State kExpectedStateOrder[] = {
      Superstructure::DISABLED_INITIALIZED,
      Superstructure::LOW_ARM_ZERO_LOWER_INTAKE,
      Superstructure::LOW_ARM_ZERO_MAYBE_LEVEL_SHOOTER,
      Superstructure::LOW_ARM_ZERO_LIFT_SHOULDER,
      Superstructure::LOW_ARM_ZERO_LEVEL_SHOOTER,
  };

  // Cycle through until arm_ and intake_ are initialized in superstructure.cc
  while (superstructure_.state() < Superstructure::DISABLED_INITIALIZED) {
    RunFor(dt());
  }

  static const int kNumberOfStates =
      sizeof(kExpectedStateOrder) / sizeof(kExpectedStateOrder[0]);

  // Next state when reached to disable
  for (int i = 0; i < kNumberOfStates; i++) {
    // Next expected state after being disabled that is expected until next
    //  state to disable at is reached
    for (int j = 0; superstructure_.state() != kExpectedStateOrder[i] && j <= i;
         j++) {
      // RunIteration until next expected state is reached with a maximum
      //  of 10000 times to ensure a breakout
      for (int o = 0;
           superstructure_.state() < kExpectedStateOrder[j] && o < 10000; o++) {
        RunFor(dt());
        EXPECT_LT(o, 9999);
      }
      EXPECT_EQ(kExpectedStateOrder[j], superstructure_.state());
    }

    EXPECT_EQ(kExpectedStateOrder[i], superstructure_.state());

    // Disable
    SetEnabled(false);
    RunFor(dt());
    SetEnabled(true);

    EXPECT_EQ(Superstructure::DISABLED_INITIALIZED, superstructure_.state());
  }

  SetEnabled(true);
  RunFor(chrono::seconds(10));
  EXPECT_EQ(Superstructure::LANDING_RUNNING, superstructure_.state());
}

// Tests that MoveButKeepBelow returns sane values.
TEST_F(SuperstructureTest, MoveButKeepBelowTest) {
  EXPECT_EQ(1.0, Superstructure::MoveButKeepBelow(1.0, 10.0, 1.0));
  EXPECT_EQ(1.0, Superstructure::MoveButKeepBelow(1.0, 2.0, 1.0));
  EXPECT_EQ(0.0, Superstructure::MoveButKeepBelow(1.0, 1.0, 1.0));
  EXPECT_EQ(1.0, Superstructure::MoveButKeepBelow(1.0, 0.0, 1.0));
}

// Tests that the integrators works.
TEST_F(SuperstructureTest, IntegratorTest) {
  SetEnabled(true);
  // Don't check for collisions for this test.
  superstructure_plant_.set_check_for_collisions(false);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.lower);
  superstructure_plant_.InitializeRelativeWristPosition(0.0);
  superstructure_plant_.set_power_error(1.0, 1.0, 1.0);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(0.0);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that zeroing while disabled works.  Starts the superstructure near a
// pulse, lets it initialize, moves it past the pulse, enables, and then make
// sure it goes to the right spot.
TEST_F(SuperstructureTest, DisabledZeroTest) {
  superstructure_plant_.InitializeIntakePosition(-0.001);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderEncoderIndexDifference * 10 - 0.001);
  superstructure_plant_.InitializeRelativeWristPosition(-0.001);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(
        constants::Values::kShoulderEncoderIndexDifference * 10);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Run disabled for 2 seconds
  RunFor(chrono::seconds(2));
  EXPECT_EQ(Superstructure::DISABLED_INITIALIZED, superstructure_.state());

  superstructure_plant_.set_power_error(1.0, 1.5, 1.0);

  RunFor(chrono::seconds(1));

  EXPECT_EQ(Superstructure::SLOW_RUNNING, superstructure_.state());
  SetEnabled(true);
  RunFor(chrono::seconds(2));

  VerifyNearGoal();
}

// Tests that the zeroing errors in the arm are caught
TEST_F(SuperstructureTest, ArmZeroingErrorTest) {
  SetEnabled(true);
  RunFor(2 * dt());
  EXPECT_NE(Superstructure::ESTOP, superstructure_.state());
  superstructure_.arm_.TriggerEstimatorError();
  RunFor(2 * dt());

  EXPECT_EQ(Superstructure::ESTOP, superstructure_.state());
}

// Tests that the zeroing errors in the intake are caught
TEST_F(SuperstructureTest, IntakeZeroingErrorTest) {
  SetEnabled(true);
  RunFor(2 * dt());
  EXPECT_NE(Superstructure::ESTOP, superstructure_.state());
  superstructure_.intake_.TriggerEstimatorError();
  RunFor(2 * dt());

  EXPECT_EQ(Superstructure::ESTOP, superstructure_.state());
}

// Tests that the loop respects shoulder acceleration limits while moving.
TEST_F(SuperstructureTest, ShoulderAccelerationLimitTest) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(1.0);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(1.5);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(1);
    goal_builder.add_max_angular_acceleration_intake(1);
    goal_builder.add_max_angular_velocity_shoulder(1);
    goal_builder.add_max_angular_acceleration_shoulder(1);
    goal_builder.add_max_angular_velocity_wrist(1);
    goal_builder.add_max_angular_acceleration_wrist(1);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // TODO(austin): The profile isn't feasible, so when we try to track it, we
  // have trouble going from the acceleration step to the constant velocity
  // step.  We end up under and then overshooting.
  superstructure_plant_.set_peak_intake_acceleration(1.10);
  superstructure_plant_.set_peak_shoulder_acceleration(1.20);
  superstructure_plant_.set_peak_wrist_acceleration(1.10);
  RunFor(chrono::seconds(6));

  VerifyNearGoal();
}

// Tests that the loop respects intake acceleration limits while moving.
TEST_F(SuperstructureTest, IntakeAccelerationLimitTest) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(1.0);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.5);
    goal_builder.add_angle_shoulder(1.0);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(1);
    goal_builder.add_max_angular_acceleration_intake(1);
    goal_builder.add_max_angular_velocity_shoulder(1);
    goal_builder.add_max_angular_acceleration_shoulder(1);
    goal_builder.add_max_angular_velocity_wrist(1);
    goal_builder.add_max_angular_acceleration_wrist(1);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_peak_intake_acceleration(1.20);
  superstructure_plant_.set_peak_shoulder_acceleration(1.20);
  superstructure_plant_.set_peak_wrist_acceleration(1.20);
  RunFor(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects wrist acceleration limits while moving.
TEST_F(SuperstructureTest, WristAccelerationLimitTest) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(
        CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference + 0.1);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(
        CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference + 0.1);
    goal_builder.add_angle_wrist(0.5);
    goal_builder.add_max_angular_velocity_intake(1);
    goal_builder.add_max_angular_acceleration_intake(1);
    goal_builder.add_max_angular_velocity_shoulder(1);
    goal_builder.add_max_angular_acceleration_shoulder(1);
    goal_builder.add_max_angular_velocity_wrist(1);
    goal_builder.add_max_angular_acceleration_wrist(1);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_peak_intake_acceleration(1.05);
  superstructure_plant_.set_peak_shoulder_acceleration(1.05);
  superstructure_plant_.set_peak_wrist_acceleration(1.05);
  RunFor(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects intake handles saturation while accelerating
// correctly.
TEST_F(SuperstructureTest, SaturatedIntakeProfileTest) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(
        CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.5);
    goal_builder.add_angle_shoulder(
        CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(4.5);
    goal_builder.add_max_angular_acceleration_intake(800);
    goal_builder.add_max_angular_velocity_shoulder(1);
    goal_builder.add_max_angular_acceleration_shoulder(100);
    goal_builder.add_max_angular_velocity_wrist(1);
    goal_builder.add_max_angular_acceleration_wrist(100);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_peak_intake_velocity(4.65);
  superstructure_plant_.set_peak_shoulder_velocity(1.00);
  superstructure_plant_.set_peak_wrist_velocity(1.00);
  RunFor(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects shoulder handles saturation while accelerating
// correctly.
TEST_F(SuperstructureTest, SaturatedShoulderProfileTest) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(1.0);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(1.9);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(1.0);
    goal_builder.add_max_angular_acceleration_intake(1.0);
    goal_builder.add_max_angular_velocity_shoulder(5.0);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(1);
    goal_builder.add_max_angular_acceleration_wrist(100);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_peak_intake_velocity(1.0);
  superstructure_plant_.set_peak_shoulder_velocity(5.5);
  superstructure_plant_.set_peak_wrist_velocity(1.0);
  RunFor(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects wrist handles saturation while accelerating
// correctly.
TEST_F(SuperstructureTest, SaturatedWristProfileTest) {
  SetEnabled(true);
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(
        CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference + 0.1);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(
        CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference + 0.1);
    goal_builder.add_angle_wrist(1.3);
    goal_builder.add_max_angular_velocity_intake(1.0);
    goal_builder.add_max_angular_acceleration_intake(1.0);
    goal_builder.add_max_angular_velocity_shoulder(1.0);
    goal_builder.add_max_angular_acceleration_shoulder(1.0);
    goal_builder.add_max_angular_velocity_wrist(10.0);
    goal_builder.add_max_angular_acceleration_wrist(160.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  superstructure_plant_.set_peak_intake_velocity(1.0);
  superstructure_plant_.set_peak_shoulder_velocity(1.0);
  superstructure_plant_.set_peak_wrist_velocity(10.2);
  RunFor(chrono::seconds(4));

  VerifyNearGoal();
}

// Make sure that the intake moves out of the way when the arm wants to move
// into a shooting position.
TEST_F(SuperstructureTest, AvoidCollisionWhenMovingArmFromStart) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(0.0);
  superstructure_plant_.InitializeAbsoluteWristPosition(0.0);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(
        constants::Values::kIntakeRange.upper);  // stowed
    goal_builder.add_angle_shoulder(
        constants::Values::kShoulderRange.lower);  // Down
    goal_builder.add_angle_wrist(0.0);             // Stowed
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(15));

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(
        constants::Values::kIntakeRange.upper);   // stowed
    goal_builder.add_angle_shoulder(M_PI / 4.0);  // in the collision area
    goal_builder.add_angle_wrist(M_PI / 2.0);     // down
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));

  superstructure_status_fetcher_.Fetch();
  ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);

  // The intake should be out of the way despite being told to move to stowing.
  EXPECT_LT(superstructure_status_fetcher_->intake()->angle(), M_PI);
  EXPECT_LT(superstructure_status_fetcher_->intake()->angle(),
            constants::Values::kIntakeRange.upper);
  EXPECT_LT(superstructure_status_fetcher_->intake()->angle(),
            CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference);

  // The arm should have reached its goal.
  EXPECT_NEAR(M_PI / 4.0, superstructure_status_fetcher_->shoulder()->angle(),
              0.001);

  // The wrist should be forced into a stowing position.
  // Since the intake is kicked out, we can be within
  // kMaxWristAngleForMovingByIntake
  EXPECT_NEAR(0, superstructure_status_fetcher_->wrist()->angle(),
              CollisionAvoidance::kMaxWristAngleForMovingByIntake + 0.001);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(
        constants::Values::kIntakeRange.upper);   // stowed
    goal_builder.add_angle_shoulder(M_PI / 2.0);  // in the collision area
    goal_builder.add_angle_wrist(M_PI);           // forward
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(5));
  VerifyNearGoal();
}

// Make sure that the shooter holds itself level when the arm comes down
// into a stowing/intaking position.
TEST_F(SuperstructureTest, AvoidCollisionWhenStowingArm) {
  SetEnabled(true);
  superstructure_plant_.InitializeIntakePosition(0.0);           // forward
  superstructure_plant_.InitializeShoulderPosition(M_PI / 2.0);  // up
  superstructure_plant_.InitializeAbsoluteWristPosition(M_PI);   // forward

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(0.0);
    goal_builder.add_angle_wrist(M_PI);  // intentionally asking for forward
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(15));

  superstructure_status_fetcher_.Fetch();
  ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr);

  // The intake should be in intaking position, as asked.
  EXPECT_NEAR(0.0, superstructure_status_fetcher_->intake()->angle(), 0.001);

  // The shoulder and wrist should both be at zero degrees (i.e.
  // stowed/intaking position).
  EXPECT_NEAR(0.0, superstructure_status_fetcher_->shoulder()->angle(), 0.001);
  EXPECT_NEAR(0.0, superstructure_status_fetcher_->wrist()->angle(), 0.001);
}

// Make sure that we can properly detect a collision.
TEST_F(SuperstructureTest, DetectAndFixCollisionBetweenArmAndIntake) {
  SetEnabled(true);
  // Zero & go straight up with the shoulder.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(M_PI * 0.5);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  VerifyNearGoal();

  // Since we're explicitly checking for collisions, we don't want to fail the
  // test because of collisions.
  superstructure_plant_.set_check_for_collisions(false);

  // Move shoulder down until collided by applying a voltage offset while
  // disabled.
  superstructure_plant_.set_power_error(0.0, -1.0, 0.0);
  SetEnabled(false);
  while (!superstructure_plant_.collided()) {
    RunFor(dt());
  }
  RunFor(chrono::milliseconds(500));  // Move a bit further down.

  ASSERT_TRUE(superstructure_plant_.collided());
  EXPECT_EQ(Superstructure::SLOW_RUNNING, superstructure_.state());
  superstructure_plant_.set_power_error(0.0, 0.0, 0.0);

  // Make sure that the collision avoidance will properly move the limbs out of
  // the collision area.
  SetEnabled(true);
  RunFor(chrono::seconds(10));
  ASSERT_FALSE(superstructure_plant_.collided());
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
}

// Make sure that we can properly detect a collision.
TEST_F(SuperstructureTest, DetectAndFixShoulderInDrivebase) {
  SetEnabled(true);
  // Zero & go straight up with the shoulder.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.lower);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  VerifyNearGoal();

  // Since we're explicitly checking for collisions, we don't want to fail the
  // test because of collisions.
  superstructure_plant_.set_check_for_collisions(false);

  // Move wrist up until on top of the bellypan
  superstructure_plant_.set_power_error(0.0, 0.0, -1.0);
  SetEnabled(false);
  while (superstructure_plant_.wrist_angle() > -0.2) {
    RunFor(dt());
  }

  ASSERT_TRUE(superstructure_plant_.collided());
  EXPECT_EQ(Superstructure::LANDING_SLOW_RUNNING, superstructure_.state());

  // Make sure that the collision avoidance will properly move the limbs out of
  // the collision area.
  superstructure_plant_.set_power_error(0.0, 0.0, 0.0);
  SetEnabled(true);
  RunFor(chrono::seconds(3));
  ASSERT_FALSE(superstructure_plant_.collided());
  EXPECT_EQ(Superstructure::LANDING_RUNNING, superstructure_.state());
}

// Make sure that the landing voltage limit works.
TEST_F(SuperstructureTest, LandingDownVoltageLimit) {
  SetEnabled(true);

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(0.0);
  superstructure_plant_.InitializeAbsoluteWristPosition(0.0);

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(constants::Values::kShoulderRange.lower);
    goal_builder.add_angle_wrist(0.0);  // intentionally asking for forward
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(6));
  VerifyNearGoal();

  // If we are near the bottom of the range, we won't have enough power to
  // compensate for the offset.  This means that we fail if we get to the goal.
  superstructure_plant_.set_power_error(
      0.0, -Superstructure::kLandingShoulderDownVoltage, 0.0);
  RunFor(chrono::seconds(2));
  superstructure_plant_.set_power_error(
      0.0, -2.0 * Superstructure::kLandingShoulderDownVoltage, 0.0);
  RunFor(chrono::seconds(2));
  superstructure_goal_fetcher_.Fetch();
  EXPECT_LE(constants::Values::kShoulderRange.lower,
            superstructure_goal_fetcher_->angle_shoulder());
}

// Make sure that we land slowly.
TEST_F(SuperstructureTest, LandSlowly) {
  SetEnabled(true);
  // Zero & go to initial position.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(M_PI * 0.25);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(8));

  // Tell it to land in the bellypan as fast as possible.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(0.0);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Wait until we hit the transition point.
  do {
    RunFor(dt());
    superstructure_status_fetcher_.Fetch();
  } while (superstructure_plant_.shoulder_angle() >
           Superstructure::kShoulderTransitionToLanded);

  superstructure_plant_.set_peak_shoulder_velocity(0.55);
  RunFor(chrono::seconds(4));
}

// Make sure that we quickly take off from a land.
TEST_F(SuperstructureTest, TakeOffQuickly) {
  SetEnabled(true);
  // Zero & go to initial position.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(0.0);
    goal_builder.add_angle_wrist(0.0);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(8));

  // Tell it to take off as fast as possible.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angle_intake(0.0);
    goal_builder.add_angle_shoulder(M_PI / 2.0);
    goal_builder.add_angle_wrist(0.0);
    goal_builder.add_max_angular_velocity_intake(20);
    goal_builder.add_max_angular_acceleration_intake(20);
    goal_builder.add_max_angular_velocity_shoulder(20);
    goal_builder.add_max_angular_acceleration_shoulder(20);
    goal_builder.add_max_angular_velocity_wrist(20);
    goal_builder.add_max_angular_acceleration_wrist(20);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Wait until we hit the transition point.
  do {
    RunFor(dt());
    superstructure_status_fetcher_.Fetch();
  } while (superstructure_plant_.shoulder_angle() <
           Superstructure::kShoulderTransitionToLanded);

  // Make sure we are faster than the limited speed (which would indicate that
  // we are still holding the shoulder back even when it's taking off).
  EXPECT_GE(superstructure_plant_.shoulder_angular_velocity(), 0.55);
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016
