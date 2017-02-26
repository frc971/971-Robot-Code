#include "y2016/control_loops/superstructure/superstructure.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/common/commonmath.h"
#include "aos/common/controls/control_loop_test.h"
#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2016/control_loops/superstructure/arm_plant.h"
#include "y2016/control_loops/superstructure/intake_plant.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"

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
  SuperstructureSimulation()
      : intake_plant_(new IntakePlant(MakeIntakePlant())),
        arm_plant_(new ArmPlant(MakeArmPlant())),
        pot_encoder_intake_(constants::Values::kIntakeEncoderIndexDifference),
        pot_encoder_shoulder_(
            constants::Values::kShoulderEncoderIndexDifference),
        pot_encoder_wrist_(constants::Values::kWristEncoderIndexDifference),
        superstructure_queue_(".y2016.control_loops.superstructure", 0x0,
                              ".y2016.control_loops.superstructure.goal",
                              ".y2016.control_loops.superstructure.status",
                              ".y2016.control_loops.superstructure.output",
                              ".y2016.control_loops.superstructure.status") {
    InitializeIntakePosition(0.0);
    InitializeShoulderPosition(0.0);
    InitializeRelativeWristPosition(0.0);
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
    ::aos::ScopedMessagePtr<control_loops::SuperstructureQueue::Position>
        position = superstructure_queue_.position.MakeMessage();

    pot_encoder_intake_.GetSensorValues(&position->intake);
    pot_encoder_shoulder_.GetSensorValues(&position->shoulder);
    pot_encoder_wrist_.GetSensorValues(&position->wrist);

    position.Send();
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
    EXPECT_TRUE(superstructure_queue_.output.FetchLatest());

    // Feed voltages into physics simulation.
    ::Eigen::Matrix<double, 1, 1> intake_U;
    ::Eigen::Matrix<double, 2, 1> arm_U;
    intake_U << superstructure_queue_.output->voltage_intake +
                    intake_plant_->voltage_offset();

    arm_U << superstructure_queue_.output->voltage_shoulder +
                 arm_plant_->shoulder_voltage_offset(),
        superstructure_queue_.output->voltage_wrist +
            arm_plant_->wrist_voltage_offset();

    // Verify that the correct power limits are being respected depending on
    // which mode we are in.
    EXPECT_TRUE(superstructure_queue_.status.FetchLatest());
    if (superstructure_queue_.status->state == Superstructure::RUNNING ||
        superstructure_queue_.status->state ==
            Superstructure::LANDING_RUNNING) {
      CHECK_LE(::std::abs(superstructure_queue_.output->voltage_intake),
               Superstructure::kOperatingVoltage);
      CHECK_LE(::std::abs(superstructure_queue_.output->voltage_shoulder),
               Superstructure::kOperatingVoltage);
      CHECK_LE(::std::abs(superstructure_queue_.output->voltage_wrist),
               Superstructure::kOperatingVoltage);
    } else {
      CHECK_LE(::std::abs(superstructure_queue_.output->voltage_intake),
               Superstructure::kZeroingVoltage);
      CHECK_LE(::std::abs(superstructure_queue_.output->voltage_shoulder),
               Superstructure::kZeroingVoltage);
      CHECK_LE(::std::abs(superstructure_queue_.output->voltage_wrist),
               Superstructure::kZeroingVoltage);
    }
    if (arm_plant_->X(0, 0) <=
        Superstructure::kShoulderTransitionToLanded + 1e-4) {
      CHECK_GE(superstructure_queue_.output->voltage_shoulder,
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
  }

 private:
  ::std::unique_ptr<IntakePlant> intake_plant_;
  ::std::unique_ptr<ArmPlant> arm_plant_;

  PositionSensorSimulator pot_encoder_intake_;
  PositionSensorSimulator pot_encoder_shoulder_;
  PositionSensorSimulator pot_encoder_wrist_;

  SuperstructureQueue superstructure_queue_;
};

class SuperstructureTest : public ::aos::testing::ControlLoopTest {
 protected:
  SuperstructureTest()
      : superstructure_queue_(".y2016.control_loops.superstructure", 0x0,
                              ".y2016.control_loops.superstructure.goal",
                              ".y2016.control_loops.superstructure.status",
                              ".y2016.control_loops.superstructure.output",
                              ".y2016.control_loops.superstructure.status"),
        superstructure_(&superstructure_queue_),
        superstructure_plant_() {}

  void VerifyNearGoal() {
    superstructure_queue_.goal.FetchLatest();
    superstructure_queue_.status.FetchLatest();

    EXPECT_TRUE(superstructure_queue_.goal.get() != nullptr);
    EXPECT_TRUE(superstructure_queue_.status.get() != nullptr);

    EXPECT_NEAR(superstructure_queue_.goal->angle_intake,
                superstructure_queue_.status->intake.angle, 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->angle_shoulder,
                superstructure_queue_.status->shoulder.angle, 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->angle_wrist,
                superstructure_queue_.status->wrist.angle, 0.001);

    EXPECT_NEAR(superstructure_queue_.goal->angle_intake,
                superstructure_plant_.intake_angle(), 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->angle_shoulder,
                superstructure_plant_.shoulder_angle(), 0.001);
    EXPECT_NEAR(superstructure_queue_.goal->angle_wrist,
                superstructure_plant_.wrist_angle(), 0.001);
  }

  // Runs one iteration of the whole simulation and checks that separation
  // remains reasonable.
  void RunIteration(bool enabled = true) {
    SendMessages(enabled);

    superstructure_plant_.SendPositionMessage();
    superstructure_.Iterate();
    superstructure_plant_.Simulate();

    TickTime();
  }

  // Runs iterations until the specified amount of simulated time has elapsed.
  void RunForTime(monotonic_clock::duration run_for, bool enabled = true) {
    const auto start_time = monotonic_clock::now();
    while (monotonic_clock::now() < start_time + run_for) {
      const auto loop_start_time = monotonic_clock::now();
      double begin_shoulder_velocity =
          superstructure_plant_.shoulder_angular_velocity();
      double begin_intake_velocity =
          superstructure_plant_.intake_angular_velocity();
      double begin_wrist_velocity =
          superstructure_plant_.wrist_angular_velocity();
      RunIteration(enabled);
      const double loop_time =
          chrono::duration_cast<chrono::duration<double>>(
              monotonic_clock::now() - loop_start_time).count();
      const double shoulder_acceleration =
          (superstructure_plant_.shoulder_angular_velocity() -
           begin_shoulder_velocity) /
          loop_time;
      const double intake_acceleration =
          (superstructure_plant_.intake_angular_velocity() -
           begin_intake_velocity) /
          loop_time;
      const double wrist_acceleration =
          (superstructure_plant_.wrist_angular_velocity() -
           begin_wrist_velocity) /
          loop_time;
      EXPECT_GE(peak_shoulder_acceleration_, shoulder_acceleration);
      EXPECT_LE(-peak_shoulder_acceleration_, shoulder_acceleration);
      EXPECT_GE(peak_intake_acceleration_, intake_acceleration);
      EXPECT_LE(-peak_intake_acceleration_, intake_acceleration);
      EXPECT_GE(peak_wrist_acceleration_, wrist_acceleration);
      EXPECT_LE(-peak_wrist_acceleration_, wrist_acceleration);

      EXPECT_GE(peak_shoulder_velocity_,
                superstructure_plant_.shoulder_angular_velocity());
      EXPECT_LE(-peak_shoulder_velocity_,
                superstructure_plant_.shoulder_angular_velocity());
      EXPECT_GE(peak_intake_velocity_,
                superstructure_plant_.intake_angular_velocity());
      EXPECT_LE(-peak_intake_velocity_,
                superstructure_plant_.intake_angular_velocity());
      EXPECT_GE(peak_wrist_velocity_,
                superstructure_plant_.wrist_angular_velocity());
      EXPECT_LE(-peak_wrist_velocity_,
                superstructure_plant_.wrist_angular_velocity());

      if (check_for_collisions_) {
        ASSERT_FALSE(collided());
      }
    }
  }

  // Helper function to quickly check if either the estimation detected a
  // collision or if there's a collision using ground-truth plant values.
  bool collided() const {
    return superstructure_.collided() ||
           CollisionAvoidance::collided_with_given_angles(
               superstructure_plant_.shoulder_angle(),
               superstructure_plant_.wrist_angle(),
               superstructure_plant_.intake_angle());
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

  bool check_for_collisions_ = true;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointed to
  // shared memory that is no longer valid.
  SuperstructureQueue superstructure_queue_;

  // Create a control loop and simulation.
  Superstructure superstructure_;
  SuperstructureSimulation superstructure_plant_;

 private:
  // The acceleration limits to check for while moving for the 3 axes.
  double peak_intake_acceleration_ = 1e10;
  double peak_shoulder_acceleration_ = 1e10;
  double peak_wrist_acceleration_ = 1e10;
  // The velocity limits to check for while moving for the 3 axes.
  double peak_intake_velocity_ = 1e10;
  double peak_shoulder_velocity_ = 1e10;
  double peak_wrist_velocity_ = 1e10;
};

// Tests that the superstructure does nothing when the goal is zero.
TEST_F(SuperstructureTest, DoesNothing) {
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0)
                  .angle_shoulder(0)
                  .angle_wrist(0)
                  .max_angular_velocity_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  // TODO(phil): Send a goal of some sort.
  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Tests that the loop can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  // Set a reasonable goal.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(M_PI / 4.0)
                  .angle_shoulder(1.4)
                  .angle_wrist(M_PI / 4.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  // Give it a lot of time to get there.
  RunForTime(chrono::seconds(8));

  VerifyNearGoal();
}

// Tests that the loop doesn't try and go beyond the physical range of the
// mechanisms.
TEST_F(SuperstructureTest, RespectsRange) {
  // Set some ridiculous goals to test upper limits.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(M_PI * 10)
                  .angle_shoulder(M_PI * 10)
                  .angle_wrist(M_PI * 10)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());
  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();
  EXPECT_NEAR(constants::Values::kIntakeRange.upper,
              superstructure_queue_.status->intake.angle, 0.001);
  EXPECT_NEAR(constants::Values::kShoulderRange.upper,
              superstructure_queue_.status->shoulder.angle, 0.001);
  EXPECT_NEAR(constants::Values::kWristRange.upper +
                  constants::Values::kShoulderRange.upper,
              superstructure_queue_.status->wrist.angle, 0.001);

  // Set some ridiculous goals to test limits.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(M_PI * 10)
                  .angle_shoulder(M_PI * 10)
                  .angle_wrist(-M_PI * 10.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();
  EXPECT_NEAR(constants::Values::kIntakeRange.upper,
              superstructure_queue_.status->intake.angle, 0.001);
  EXPECT_NEAR(constants::Values::kShoulderRange.upper,
              superstructure_queue_.status->shoulder.angle, 0.001);
  EXPECT_NEAR(constants::Values::kWristRange.lower +
                  constants::Values::kShoulderRange.upper,
              superstructure_queue_.status->wrist.angle, 0.001);

  // Set some ridiculous goals to test lower limits.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(-M_PI * 10)
                  .angle_shoulder(-M_PI * 10)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  RunForTime(chrono::seconds(10));

  // Check that we are near our soft limit.
  superstructure_queue_.status.FetchLatest();
  EXPECT_NEAR(constants::Values::kIntakeRange.lower,
              superstructure_queue_.status->intake.angle, 0.001);
  EXPECT_NEAR(constants::Values::kShoulderRange.lower,
              superstructure_queue_.status->shoulder.angle, 0.001);
  EXPECT_NEAR(0.0, superstructure_queue_.status->wrist.angle, 0.001);
}

// Tests that the loop zeroes when run for a while.
TEST_F(SuperstructureTest, ZeroTest) {
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(constants::Values::kIntakeRange.lower)
                  .angle_shoulder(constants::Values::kShoulderRange.lower)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  RunForTime(chrono::seconds(10));

  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  RunForTime(chrono::seconds(5));

  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
}

// Tests that starting at the lower hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, LowerHardstopStartup) {
  // Don't check for collisions for this test.
  check_for_collisions_ = false;

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.lower);
  superstructure_plant_.InitializeRelativeWristPosition(
      constants::Values::kWristRange.lower);
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(constants::Values::kIntakeRange.upper)
                  .angle_shoulder(constants::Values::kShoulderRange.upper)
                  .angle_wrist(constants::Values::kWristRange.upper +
                               constants::Values::kShoulderRange.upper)
                  .Send());
  // We have to wait for it to put the elevator in a safe position as well.
  RunForTime(chrono::seconds(15));

  VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TEST_F(SuperstructureTest, UpperHardstopStartup) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.upper);
  superstructure_plant_.InitializeRelativeWristPosition(
      constants::Values::kWristRange.upper);
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(constants::Values::kIntakeRange.lower)
                  .angle_shoulder(constants::Values::kShoulderRange.lower)
                  .angle_wrist(0.0)
                  .Send());
  // We have to wait for it to put the superstructure in a safe position as
  // well.
  RunForTime(chrono::seconds(20));

  VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TEST_F(SuperstructureTest, ResetTest) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.upper);
  superstructure_plant_.InitializeRelativeWristPosition(
      constants::Values::kWristRange.upper);

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(constants::Values::kIntakeRange.lower + 0.3)
                  .angle_shoulder(constants::Values::kShoulderRange.upper)
                  .angle_wrist(0.0)
                  .Send());
  RunForTime(chrono::seconds(15));

  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
  VerifyNearGoal();
  SimulateSensorReset();
  RunForTime(chrono::milliseconds(100));
  EXPECT_NE(Superstructure::RUNNING, superstructure_.state());
  RunForTime(chrono::milliseconds(10000));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
  VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TEST_F(SuperstructureTest, DisabledGoalTest) {
  // Don't check for collisions for this test.
  check_for_collisions_ = false;

  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(constants::Values::kIntakeRange.lower + 0.03)
          .angle_shoulder(constants::Values::kShoulderRange.lower + 0.03)
          .angle_wrist(constants::Values::kWristRange.lower + 0.03)
          .Send());

  RunForTime(chrono::milliseconds(100), false);
  EXPECT_EQ(0.0, superstructure_.intake_.goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.arm_.goal(0, 0));
  EXPECT_EQ(0.0, superstructure_.arm_.goal(2, 0));

  // Now make sure they move correctly
  RunForTime(chrono::milliseconds(4000), true);
  EXPECT_NE(0.0, superstructure_.intake_.goal(0, 0));
  EXPECT_NE(0.0, superstructure_.arm_.goal(0, 0));
  EXPECT_NE(0.0, superstructure_.arm_.goal(2, 0));
}

// Tests that disabling while zeroing at any state restarts from beginning
TEST_F(SuperstructureTest, DisabledWhileZeroingHigh) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.upper);
  superstructure_plant_.InitializeAbsoluteWristPosition(
      constants::Values::kWristRange.upper);

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(constants::Values::kIntakeRange.upper)
                  .angle_shoulder(constants::Values::kShoulderRange.upper)
                  .angle_wrist(constants::Values::kWristRange.upper)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

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
    RunIteration(true);
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
        RunIteration(true);
      }
      EXPECT_EQ(kExpectedStateOrder[j], superstructure_.state());
    }

    EXPECT_EQ(kExpectedStateOrder[i], superstructure_.state());

    // Disable
    RunIteration(false);

    EXPECT_EQ(Superstructure::DISABLED_INITIALIZED, superstructure_.state());
  }

  RunForTime(chrono::seconds(10));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
}

// Tests that disabling while zeroing at any state restarts from beginning
TEST_F(SuperstructureTest, DisabledWhileZeroingLow) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.lower);
  superstructure_plant_.InitializeAbsoluteWristPosition(0.0);

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(constants::Values::kIntakeRange.lower)
                  .angle_shoulder(constants::Values::kShoulderRange.lower)
                  .angle_wrist(constants::Values::kWristRange.lower)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

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
    RunIteration(true);
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
        RunIteration(true);
      }
      EXPECT_EQ(kExpectedStateOrder[j], superstructure_.state());
    }

    EXPECT_EQ(kExpectedStateOrder[i], superstructure_.state());

    // Disable
    RunIteration(false);

    EXPECT_EQ(Superstructure::DISABLED_INITIALIZED, superstructure_.state());
  }

  RunForTime(chrono::seconds(10));
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
  // Don't check for collisions for this test.
  check_for_collisions_ = false;

  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(
      constants::Values::kShoulderRange.lower);
  superstructure_plant_.InitializeRelativeWristPosition(0.0);
  superstructure_plant_.set_power_error(1.0, 1.0, 1.0);
  superstructure_queue_.goal.MakeWithBuilder()
      .angle_intake(0.0)
      .angle_shoulder(0.0)
      .angle_wrist(0.0)
      .Send();

  RunForTime(chrono::seconds(8));

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

  superstructure_queue_.goal.MakeWithBuilder()
      .angle_intake(0.0)
      .angle_shoulder(constants::Values::kShoulderEncoderIndexDifference * 10)
      .angle_wrist(0.0)
      .Send();

  // Run disabled for 2 seconds
  RunForTime(chrono::seconds(2), false);
  EXPECT_EQ(Superstructure::DISABLED_INITIALIZED, superstructure_.state());

  superstructure_plant_.set_power_error(1.0, 1.5, 1.0);

  RunForTime(chrono::seconds(1), false);

  EXPECT_EQ(Superstructure::SLOW_RUNNING, superstructure_.state());
  RunForTime(chrono::seconds(2), true);

  VerifyNearGoal();
}

// Tests that the zeroing errors in the arm are caught
TEST_F(SuperstructureTest, ArmZeroingErrorTest) {
  RunIteration();
  EXPECT_NE(Superstructure::ESTOP, superstructure_.state());
  superstructure_.arm_.TriggerEstimatorError();
  RunIteration();

  EXPECT_EQ(Superstructure::ESTOP, superstructure_.state());
}

// Tests that the zeroing errors in the intake are caught
TEST_F(SuperstructureTest, IntakeZeroingErrorTest) {
  RunIteration();
  EXPECT_NE(Superstructure::ESTOP, superstructure_.state());
  superstructure_.intake_.TriggerEstimatorError();
  RunIteration();

  EXPECT_EQ(Superstructure::ESTOP, superstructure_.state());
}

// Tests that the loop respects shoulder acceleration limits while moving.
TEST_F(SuperstructureTest, ShoulderAccelerationLimitTest) {
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(1.0)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  RunForTime(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(1.5)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(1)
                  .max_angular_acceleration_intake(1)
                  .max_angular_velocity_shoulder(1)
                  .max_angular_acceleration_shoulder(1)
                  .max_angular_velocity_wrist(1)
                  .max_angular_acceleration_wrist(1)
                  .Send());

  // TODO(austin): The profile isn't feasible, so when we try to track it, we
  // have trouble going from the acceleration step to the constant velocity
  // step.  We end up under and then overshooting.
  set_peak_intake_acceleration(1.10);
  set_peak_shoulder_acceleration(1.20);
  set_peak_wrist_acceleration(1.10);
  RunForTime(chrono::seconds(6));

  VerifyNearGoal();
}

// Tests that the loop respects intake acceleration limits while moving.
TEST_F(SuperstructureTest, IntakeAccelerationLimitTest) {
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(1.0)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  RunForTime(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.5)
                  .angle_shoulder(1.0)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(1)
                  .max_angular_acceleration_intake(1)
                  .max_angular_velocity_shoulder(1)
                  .max_angular_acceleration_shoulder(1)
                  .max_angular_velocity_wrist(1)
                  .max_angular_acceleration_wrist(1)
                  .Send());

  set_peak_intake_acceleration(1.20);
  set_peak_shoulder_acceleration(1.20);
  set_peak_wrist_acceleration(1.20);
  RunForTime(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects wrist acceleration limits while moving.
TEST_F(SuperstructureTest, WristAccelerationLimitTest) {
  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(0.0)
          .angle_shoulder(
               CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference +
               0.1)
          .angle_wrist(0.0)
          .max_angular_velocity_intake(20)
          .max_angular_acceleration_intake(20)
          .max_angular_velocity_shoulder(20)
          .max_angular_acceleration_shoulder(20)
          .max_angular_velocity_wrist(20)
          .max_angular_acceleration_wrist(20)
          .Send());

  RunForTime(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(0.0)
          .angle_shoulder(
               CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference +
               0.1)
          .angle_wrist(0.5)
          .max_angular_velocity_intake(1)
          .max_angular_acceleration_intake(1)
          .max_angular_velocity_shoulder(1)
          .max_angular_acceleration_shoulder(1)
          .max_angular_velocity_wrist(1)
          .max_angular_acceleration_wrist(1)
          .Send());

  set_peak_intake_acceleration(1.05);
  set_peak_shoulder_acceleration(1.05);
  set_peak_wrist_acceleration(1.05);
  RunForTime(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects intake handles saturation while accelerating
// correctly.
TEST_F(SuperstructureTest, SaturatedIntakeProfileTest) {
  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(0.0)
          .angle_shoulder(
               CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference)
          .angle_wrist(0.0)
          .max_angular_velocity_intake(20)
          .max_angular_acceleration_intake(20)
          .max_angular_velocity_shoulder(20)
          .max_angular_acceleration_shoulder(20)
          .max_angular_velocity_wrist(20)
          .max_angular_acceleration_wrist(20)
          .Send());

  RunForTime(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(0.5)
          .angle_shoulder(
               CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference)
          .angle_wrist(0.0)
          .max_angular_velocity_intake(4.5)
          .max_angular_acceleration_intake(800)
          .max_angular_velocity_shoulder(1)
          .max_angular_acceleration_shoulder(100)
          .max_angular_velocity_wrist(1)
          .max_angular_acceleration_wrist(100)
          .Send());

  set_peak_intake_velocity(4.65);
  set_peak_shoulder_velocity(1.00);
  set_peak_wrist_velocity(1.00);
  RunForTime(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects shoulder handles saturation while accelerating
// correctly.
TEST_F(SuperstructureTest, SaturatedShoulderProfileTest) {
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(1.0)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  RunForTime(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(1.9)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(1.0)
                  .max_angular_acceleration_intake(1.0)
                  .max_angular_velocity_shoulder(5.0)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(1)
                  .max_angular_acceleration_wrist(100)
                  .Send());

  set_peak_intake_velocity(1.0);
  set_peak_shoulder_velocity(5.5);
  set_peak_wrist_velocity(1.0);
  RunForTime(chrono::seconds(4));

  VerifyNearGoal();
}

// Tests that the loop respects wrist handles saturation while accelerating
// correctly.
TEST_F(SuperstructureTest, SaturatedWristProfileTest) {
  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(0.0)
          .angle_shoulder(
               CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference +
               0.1)
          .angle_wrist(0.0)
          .max_angular_velocity_intake(20)
          .max_angular_acceleration_intake(20)
          .max_angular_velocity_shoulder(20)
          .max_angular_acceleration_shoulder(20)
          .max_angular_velocity_wrist(20)
          .max_angular_acceleration_wrist(20)
          .Send());

  RunForTime(chrono::seconds(6));
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());

  VerifyNearGoal();

  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(0.0)
          .angle_shoulder(
               CollisionAvoidance::kMinShoulderAngleForIntakeUpInterference +
               0.1)
          .angle_wrist(1.3)
          .max_angular_velocity_intake(1.0)
          .max_angular_acceleration_intake(1.0)
          .max_angular_velocity_shoulder(1.0)
          .max_angular_acceleration_shoulder(1.0)
          .max_angular_velocity_wrist(10.0)
          .max_angular_acceleration_wrist(160.0)
          .Send());

  set_peak_intake_velocity(1.0);
  set_peak_shoulder_velocity(1.0);
  set_peak_wrist_velocity(10.2);
  RunForTime(chrono::seconds(4));

  VerifyNearGoal();
}

// Make sure that the intake moves out of the way when the arm wants to move
// into a shooting position.
TEST_F(SuperstructureTest, AvoidCollisionWhenMovingArmFromStart) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.upper);
  superstructure_plant_.InitializeShoulderPosition(0.0);
  superstructure_plant_.InitializeAbsoluteWristPosition(0.0);

  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(constants::Values::kIntakeRange.upper)      // stowed
          .angle_shoulder(constants::Values::kShoulderRange.lower)  // Down
          .angle_wrist(0.0)                                         // Stowed
          .Send());

  RunForTime(chrono::seconds(15));

  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(constants::Values::kIntakeRange.upper)  // stowed
          .angle_shoulder(M_PI / 4.0)  // in the collision area
          .angle_wrist(M_PI / 2.0)     // down
          .Send());

  RunForTime(chrono::seconds(5));

  superstructure_queue_.status.FetchLatest();
  ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);

  // The intake should be out of the way despite being told to move to stowing.
  EXPECT_LT(superstructure_queue_.status->intake.angle, M_PI);
  EXPECT_LT(superstructure_queue_.status->intake.angle,
            constants::Values::kIntakeRange.upper);
  EXPECT_LT(superstructure_queue_.status->intake.angle,
            CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference);

  // The arm should have reached its goal.
  EXPECT_NEAR(M_PI / 4.0, superstructure_queue_.status->shoulder.angle, 0.001);

  // The wrist should be forced into a stowing position.
  // Since the intake is kicked out, we can be within
  // kMaxWristAngleForMovingByIntake
  EXPECT_NEAR(0, superstructure_queue_.status->wrist.angle,
              CollisionAvoidance::kMaxWristAngleForMovingByIntake + 0.001);

  ASSERT_TRUE(
      superstructure_queue_.goal.MakeWithBuilder()
          .angle_intake(constants::Values::kIntakeRange.upper)  // stowed
          .angle_shoulder(M_PI / 2.0)  // in the collision area
          .angle_wrist(M_PI)     // forward
          .Send());

  RunForTime(chrono::seconds(5));
  VerifyNearGoal();
}

// Make sure that the shooter holds itself level when the arm comes down
// into a stowing/intaking position.
TEST_F(SuperstructureTest, AvoidCollisionWhenStowingArm) {
  superstructure_plant_.InitializeIntakePosition(0.0);           // forward
  superstructure_plant_.InitializeShoulderPosition(M_PI / 2.0);  // up
  superstructure_plant_.InitializeAbsoluteWristPosition(M_PI);   // forward

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(0.0)
                  .angle_wrist(M_PI)  // intentionally asking for forward
                  .Send());

  RunForTime(chrono::seconds(15));

  superstructure_queue_.status.FetchLatest();
  ASSERT_TRUE(superstructure_queue_.status.get() != nullptr);

  // The intake should be in intaking position, as asked.
  EXPECT_NEAR(0.0, superstructure_queue_.status->intake.angle, 0.001);

  // The shoulder and wrist should both be at zero degrees (i.e.
  // stowed/intaking position).
  EXPECT_NEAR(0.0, superstructure_queue_.status->shoulder.angle, 0.001);
  EXPECT_NEAR(0.0, superstructure_queue_.status->wrist.angle, 0.001);
}

// Make sure that we can properly detect a collision.
TEST_F(SuperstructureTest, DetectAndFixCollisionBetweenArmAndIntake) {
  // Zero & go straight up with the shoulder.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(M_PI * 0.5)
                  .angle_wrist(0.0)
                  .Send());

  RunForTime(chrono::seconds(6));
  VerifyNearGoal();

  // Since we're explicitly checking for collisions, we don't want to fail the
  // test because of collisions.
  check_for_collisions_ = false;

  // Move shoulder down until collided by applying a voltage offset while
  // disabled.
  superstructure_plant_.set_power_error(0.0, -1.0, 0.0);
  while (!collided()) {
    RunIteration(false);
  }
  RunForTime(chrono::milliseconds(500), false);  // Move a bit further down.

  ASSERT_TRUE(collided());
  EXPECT_EQ(Superstructure::SLOW_RUNNING, superstructure_.state());
  superstructure_plant_.set_power_error(0.0, 0.0, 0.0);

  // Make sure that the collision avoidance will properly move the limbs out of
  // the collision area.
  RunForTime(chrono::seconds(10));
  ASSERT_FALSE(collided());
  EXPECT_EQ(Superstructure::RUNNING, superstructure_.state());
}

// Make sure that we can properly detect a collision.
TEST_F(SuperstructureTest, DetectAndFixShoulderInDrivebase) {
  // Zero & go straight up with the shoulder.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(constants::Values::kShoulderRange.lower)
                  .angle_wrist(0.0)
                  .Send());

  RunForTime(chrono::seconds(6));
  VerifyNearGoal();

  // Since we're explicitly checking for collisions, we don't want to fail the
  // test because of collisions.
  check_for_collisions_ = false;

  // Move wrist up until on top of the bellypan
  superstructure_plant_.set_power_error(0.0, 0.0, -1.0);
  while (superstructure_plant_.wrist_angle() > -0.2) {
    RunIteration(false);
  }

  ASSERT_TRUE(collided());
  EXPECT_EQ(Superstructure::LANDING_SLOW_RUNNING, superstructure_.state());

  // Make sure that the collision avoidance will properly move the limbs out of
  // the collision area.
  superstructure_plant_.set_power_error(0.0, 0.0, 0.0);
  RunForTime(chrono::seconds(3));
  ASSERT_FALSE(collided());
  EXPECT_EQ(Superstructure::LANDING_RUNNING, superstructure_.state());
}

// Make sure that the landing voltage limit works.
TEST_F(SuperstructureTest, LandingDownVoltageLimit) {
  superstructure_plant_.InitializeIntakePosition(
      constants::Values::kIntakeRange.lower);
  superstructure_plant_.InitializeShoulderPosition(0.0);
  superstructure_plant_.InitializeAbsoluteWristPosition(0.0);

  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(constants::Values::kShoulderRange.lower)
                  .angle_wrist(0.0)  // intentionally asking for forward
                  .Send());

  RunForTime(chrono::seconds(6));
  VerifyNearGoal();

  // If we are near the bottom of the range, we won't have enough power to
  // compensate for the offset.  This means that we fail if we get to the goal.
  superstructure_plant_.set_power_error(
      0.0, -Superstructure::kLandingShoulderDownVoltage, 0.0);
  RunForTime(chrono::seconds(2));
  superstructure_plant_.set_power_error(
      0.0, -2.0 * Superstructure::kLandingShoulderDownVoltage, 0.0);
  RunForTime(chrono::seconds(2));
  EXPECT_LE(constants::Values::kShoulderRange.lower,
            superstructure_queue_.goal->angle_shoulder);
}

// Make sure that we land slowly.
TEST_F(SuperstructureTest, LandSlowly) {
  // Zero & go to initial position.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(M_PI * 0.25)
                  .angle_wrist(0.0)
                  .Send());
  RunForTime(chrono::seconds(8));

  // Tell it to land in the bellypan as fast as possible.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(0.0)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  // Wait until we hit the transition point.
  do {
    RunIteration();
    superstructure_queue_.status.FetchLatest();
  } while (superstructure_plant_.shoulder_angle() >
           Superstructure::kShoulderTransitionToLanded);

  set_peak_shoulder_velocity(0.55);
  RunForTime(chrono::seconds(4));
}

// Make sure that we quickly take off from a land.
TEST_F(SuperstructureTest, TakeOffQuickly) {
  // Zero & go to initial position.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(0.0)
                  .angle_wrist(0.0)
                  .Send());
  RunForTime(chrono::seconds(8));

  // Tell it to take off as fast as possible.
  ASSERT_TRUE(superstructure_queue_.goal.MakeWithBuilder()
                  .angle_intake(0.0)
                  .angle_shoulder(M_PI / 2.0)
                  .angle_wrist(0.0)
                  .max_angular_velocity_intake(20)
                  .max_angular_acceleration_intake(20)
                  .max_angular_velocity_shoulder(20)
                  .max_angular_acceleration_shoulder(20)
                  .max_angular_velocity_wrist(20)
                  .max_angular_acceleration_wrist(20)
                  .Send());

  // Wait until we hit the transition point.
  do {
    RunIteration();
    superstructure_queue_.status.FetchLatest();
  } while (superstructure_plant_.shoulder_angle() <
           Superstructure::kShoulderTransitionToLanded);

  // Make sure we are faster than the limited speed (which would indicate that
  // we are still holding the shoulder back even when it's taking off).
  EXPECT_GE(superstructure_plant_.shoulder_angular_velocity(), 0.55);
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace frc971
