#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/common/controls/control_loop_test.h"
#include "aos/common/controls/polytope.h"
#include "aos/common/network/team_number.h"
#include "aos/common/time.h"
#include "gtest/gtest.h"

#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/queues/gyro.q.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;
using ::y2016::control_loops::drivetrain::MakeDrivetrainPlant;

// TODO(Comran): Make one that doesn't depend on the actual values for a
// specific robot.
const constants::ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0,  0.0,
                                                           0.0, 0.25, 0.75};

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,

      ::y2016::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeKFDrivetrainLoop,

      ::y2016::control_loops::drivetrain::kDt,
      ::y2016::control_loops::drivetrain::kRobotRadius,
      ::y2016::control_loops::drivetrain::kWheelRadius,
      ::y2016::control_loops::drivetrain::kV,

      ::y2016::control_loops::drivetrain::kHighGearRatio,
      ::y2016::control_loops::drivetrain::kLowGearRatio,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      false,
      0,

      0.25,
      1.00};

  return kDrivetrainConfig;
};

class DrivetrainPlant : public StateFeedbackPlant<4, 2, 2> {
 public:
  explicit DrivetrainPlant(StateFeedbackPlant<4, 2, 2> &&other)
      : StateFeedbackPlant<4, 2, 2>(::std::move(other)) {}

  void CheckU(const Eigen::Matrix<double, 2, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + left_voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + left_voltage_offset_);
    EXPECT_LE(U(1, 0), U_max(1, 0) + 0.00001 + right_voltage_offset_);
    EXPECT_GE(U(1, 0), U_min(1, 0) - 0.00001 + right_voltage_offset_);
  }

  double left_voltage_offset() const { return left_voltage_offset_; }
  void set_left_voltage_offset(double left_voltage_offset) {
    left_voltage_offset_ = left_voltage_offset;
  }

  double right_voltage_offset() const { return right_voltage_offset_; }
  void set_right_voltage_offset(double right_voltage_offset) {
    right_voltage_offset_ = right_voltage_offset;
  }

 private:
  double left_voltage_offset_ = 0.0;
  double right_voltage_offset_ = 0.0;
};

// Class which simulates the drivetrain and sends out queue messages containing
// the position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation()
      : drivetrain_plant_(new DrivetrainPlant(MakeDrivetrainPlant())),
        my_drivetrain_queue_(".frc971.control_loops.drivetrain", 0x8a8dde77,
                             ".frc971.control_loops.drivetrain.goal",
                             ".frc971.control_loops.drivetrain.position",
                             ".frc971.control_loops.drivetrain.output",
                             ".frc971.control_loops.drivetrain.status"),
        gyro_reading_(::frc971::sensors::gyro_reading.name()) {
    Reinitialize();
    last_U_.setZero();
  }

  // Resets the plant.
  void Reinitialize() {
    drivetrain_plant_->mutable_X(0, 0) = 0.0;
    drivetrain_plant_->mutable_X(1, 0) = 0.0;
    drivetrain_plant_->mutable_Y() =
        drivetrain_plant_->C() * drivetrain_plant_->X();
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
  }

  // Returns the position of the drivetrain.
  double GetLeftPosition() const { return drivetrain_plant_->Y(0, 0); }
  double GetRightPosition() const { return drivetrain_plant_->Y(1, 0); }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double left_encoder = GetLeftPosition();
    const double right_encoder = GetRightPosition();

    {
      ::aos::ScopedMessagePtr<
          ::frc971::control_loops::DrivetrainQueue::Position> position =
          my_drivetrain_queue_.position.MakeMessage();
      position->left_encoder = left_encoder;
      position->right_encoder = right_encoder;
      position->left_shifter_position = left_gear_high_ ? 1.0 : 0.0;
      position->right_shifter_position = right_gear_high_ ? 1.0 : 0.0;
      position.Send();
    }

    {
      ::aos::ScopedMessagePtr<::frc971::sensors::GyroReading> gyro =
          gyro_reading_.MakeMessage();
      gyro->angle = (right_encoder - left_encoder) /
                    (::y2016::control_loops::drivetrain::kRobotRadius * 2.0);
      gyro->velocity =
          (drivetrain_plant_->X(3, 0) - drivetrain_plant_->X(1, 0)) /
          (::y2016::control_loops::drivetrain::kRobotRadius * 2.0);
      gyro.Send();
    }
  }

  // Simulates the drivetrain moving for one timestep.
  void Simulate() {
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
    EXPECT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    ::Eigen::Matrix<double, 2, 1> U = last_U_;
    last_U_ << my_drivetrain_queue_.output->left_voltage,
        my_drivetrain_queue_.output->right_voltage;
    {
      const double scalar = ::aos::robot_state->voltage_battery / 12.0;
      last_U_ *= scalar;
    }
    left_gear_high_ = my_drivetrain_queue_.output->left_high;
    right_gear_high_ = my_drivetrain_queue_.output->right_high;

    if (left_gear_high_) {
      if (right_gear_high_) {
        drivetrain_plant_->set_index(3);
      } else {
        drivetrain_plant_->set_index(2);
      }
    } else {
      if (right_gear_high_) {
        drivetrain_plant_->set_index(1);
      } else {
        drivetrain_plant_->set_index(0);
      }
    }

    U(0, 0) += drivetrain_plant_->left_voltage_offset();
    U(1, 0) += drivetrain_plant_->right_voltage_offset();
    drivetrain_plant_->Update(U);
  }

  void set_left_voltage_offset(double left_voltage_offset) {
    drivetrain_plant_->set_left_voltage_offset(left_voltage_offset);
  }
  void set_right_voltage_offset(double right_voltage_offset) {
    drivetrain_plant_->set_right_voltage_offset(right_voltage_offset);
  }

  ::std::unique_ptr<DrivetrainPlant> drivetrain_plant_;

 private:
  ::frc971::control_loops::DrivetrainQueue my_drivetrain_queue_;
  ::aos::Queue<::frc971::sensors::GyroReading> gyro_reading_;

  double last_left_position_;
  double last_right_position_;

  Eigen::Matrix<double, 2, 1> last_U_;

  bool left_gear_high_ = false;
  bool right_gear_high_ = false;
};

class DrivetrainTest : public ::aos::testing::ControlLoopTest {
 protected:
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::frc971::control_loops::DrivetrainQueue my_drivetrain_queue_;

  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;

  DrivetrainTest()
      : my_drivetrain_queue_(".frc971.control_loops.drivetrain", 0x8a8dde77,
                             ".frc971.control_loops.drivetrain.goal",
                             ".frc971.control_loops.drivetrain.position",
                             ".frc971.control_loops.drivetrain.output",
                             ".frc971.control_loops.drivetrain.status"),
        drivetrain_motor_(GetDrivetrainConfig(), &my_drivetrain_queue_),
        drivetrain_motor_plant_() {
    ::frc971::sensors::gyro_reading.Clear();
    set_battery_voltage(12.0);
  }

  void RunIteration() {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  void RunForTime(monotonic_clock::duration run_for) {
    const auto end_time = monotonic_clock::now() + run_for;
    while (monotonic_clock::now() < end_time) {
      RunIteration();
    }
  }

  void VerifyNearGoal() {
    my_drivetrain_queue_.goal.FetchLatest();
    my_drivetrain_queue_.position.FetchLatest();
    EXPECT_NEAR(my_drivetrain_queue_.goal->left_goal,
                drivetrain_motor_plant_.GetLeftPosition(), 1e-3);
    EXPECT_NEAR(my_drivetrain_queue_.goal->right_goal,
                drivetrain_motor_plant_.GetRightPosition(), 1e-3);
  }

  virtual ~DrivetrainTest() { ::frc971::sensors::gyro_reading.Clear(); }
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  RunForTime(chrono::seconds(2));
  VerifyNearGoal();
}

// Tests that the drivetrain converges on a goal when under the effect of a
// voltage offset/disturbance.
TEST_F(DrivetrainTest, ConvergesWithVoltageError) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  drivetrain_motor_plant_.set_left_voltage_offset(1.0);
  drivetrain_motor_plant_.set_right_voltage_offset(1.0);
  RunForTime(chrono::milliseconds(1500));
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    if (i > 20 && i < 200) {
      SimulateTimestep(false);
    } else {
      SimulateTimestep(true);
    }
  }
  VerifyNearGoal();
}

// Tests that never having a goal doesn't break.
TEST_F(DrivetrainTest, NoGoalStart) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
  }
}

// Tests that never having a goal, but having driver's station messages, doesn't
// break.
TEST_F(DrivetrainTest, NoGoalWithRobotState) {
  RunForTime(chrono::milliseconds(100));
}

// Tests that the robot successfully drives straight forward.
// This used to not work due to a U-capping bug.
TEST_F(DrivetrainTest, DriveStraightForward) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(4.0)
      .right_goal(4.0)
      .Send();
  for (int i = 0; i < 500; ++i) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_NEAR(my_drivetrain_queue_.output->left_voltage,
                my_drivetrain_queue_.output->right_voltage, 1e-4);
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -11);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -11);
  }
  VerifyNearGoal();
}

// Tests that the robot successfully drives close to straight.
// This used to fail in simulation due to libcdd issues with U-capping.
TEST_F(DrivetrainTest, DriveAlmostStraightForward) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(4.0)
      .right_goal(3.9)
      .Send();
  for (int i = 0; i < 500; ++i) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -11);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -11);
  }
  VerifyNearGoal();
}

// Tests that converting from a left, right position to a distance, angle
// coordinate system and back returns the same answer.
TEST_F(DrivetrainTest, LinearToAngularAndBack) {
  const DrivetrainConfig dt_config = GetDrivetrainConfig();
  const double width = dt_config.robot_radius * 2.0;

  Eigen::Matrix<double, 7, 1> state;
  state << 2, 3, 4, 5, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> linear = dt_config.LeftRightToLinear(state);

  EXPECT_NEAR(3.0, linear(0, 0), 1e-6);
  EXPECT_NEAR(4.0, linear(1, 0), 1e-6);

  Eigen::Matrix<double, 2, 1> angular = dt_config.LeftRightToAngular(state);

  EXPECT_NEAR(2.0 / width, angular(0, 0), 1e-6);
  EXPECT_NEAR(2.0 / width, angular(1, 0), 1e-6);

  Eigen::Matrix<double, 4, 1> back_state =
      dt_config.AngularLinearToLeftRight(linear, angular);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(state(i, 0), back_state(i, 0), 1e-8);
  }
}

// Tests that a linear motion profile succeeds.
TEST_F(DrivetrainTest, ProfileStraightForward) {
  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->control_loop_driving = true;
    goal->left_goal = 4.0;
    goal->right_goal = 4.0;
    goal->left_velocity_goal = 0.0;
    goal->right_velocity_goal = 0.0;
    goal->linear.max_velocity = 1.0;
    goal->linear.max_acceleration = 3.0;
    goal->angular.max_velocity = 1.0;
    goal->angular.max_acceleration = 3.0;
    goal.Send();
  }

  while (monotonic_clock::now() <
         monotonic_clock::time_point(chrono::seconds(6))) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_NEAR(my_drivetrain_queue_.output->left_voltage,
                my_drivetrain_queue_.output->right_voltage, 1e-4);
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -6);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -6);
    EXPECT_LT(my_drivetrain_queue_.output->left_voltage, 6);
    EXPECT_LT(my_drivetrain_queue_.output->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that an angular motion profile succeeds.
TEST_F(DrivetrainTest, ProfileTurn) {
  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->control_loop_driving = true;
    goal->left_goal = -1.0;
    goal->right_goal = 1.0;
    goal->left_velocity_goal = 0.0;
    goal->right_velocity_goal = 0.0;
    goal->linear.max_velocity = 1.0;
    goal->linear.max_acceleration = 3.0;
    goal->angular.max_velocity = 1.0;
    goal->angular.max_acceleration = 3.0;
    goal.Send();
  }

  while (monotonic_clock::now() <
         monotonic_clock::time_point(chrono::seconds(6))) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_NEAR(my_drivetrain_queue_.output->left_voltage,
                -my_drivetrain_queue_.output->right_voltage, 1e-4);
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -6);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -6);
    EXPECT_LT(my_drivetrain_queue_.output->left_voltage, 6);
    EXPECT_LT(my_drivetrain_queue_.output->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that a mixed turn drive saturated profile succeeds.
TEST_F(DrivetrainTest, SaturatedTurnDrive) {
  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->control_loop_driving = true;
    goal->left_goal = 5.0;
    goal->right_goal = 4.0;
    goal->left_velocity_goal = 0.0;
    goal->right_velocity_goal = 0.0;
    goal->linear.max_velocity = 6.0;
    goal->linear.max_acceleration = 4.0;
    goal->angular.max_velocity = 2.0;
    goal->angular.max_acceleration = 4.0;
    goal.Send();
  }

  while (monotonic_clock::now() <
         monotonic_clock::time_point(chrono::seconds(3))) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
  }
  VerifyNearGoal();
}

// Tests that being in teleop drive for a bit and then transitioning to closed
// drive profiles nicely.
TEST_F(DrivetrainTest, OpenLoopThenClosed) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .steering(0.0)
      .throttle(1.0)
      .highgear(true)
      .quickturn(false)
      .Send();

  RunForTime(chrono::seconds(1));

  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .steering(0.0)
      .throttle(-0.3)
      .highgear(true)
      .quickturn(false)
      .Send();

  RunForTime(chrono::seconds(1));

  my_drivetrain_queue_.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .steering(0.0)
      .throttle(0.0)
      .highgear(true)
      .quickturn(false)
      .Send();

  RunForTime(chrono::seconds(10));

  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->control_loop_driving = true;
    goal->left_goal = 5.0;
    goal->right_goal = 4.0;
    goal->left_velocity_goal = 0.0;
    goal->right_velocity_goal = 0.0;
    goal->linear.max_velocity = 1.0;
    goal->linear.max_acceleration = 2.0;
    goal->angular.max_velocity = 1.0;
    goal->angular.max_acceleration = 2.0;
    goal.Send();
  }

  const auto end_time = monotonic_clock::now() + chrono::seconds(4);
  while (monotonic_clock::now() < end_time) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -6);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -6);
    EXPECT_LT(my_drivetrain_queue_.output->left_voltage, 6);
    EXPECT_LT(my_drivetrain_queue_.output->right_voltage, 6);
  }
  VerifyNearGoal();
}

::aos::controls::HVPolytope<2, 4, 4> MakeBox(double x1_min, double x1_max,
                                             double x2_min, double x2_max) {
  Eigen::Matrix<double, 4, 2> box_H;
  box_H << /*[[*/ 1.0, 0.0 /*]*/,
      /*[*/ -1.0, 0.0 /*]*/,
      /*[*/ 0.0, 1.0 /*]*/,
      /*[*/ 0.0, -1.0 /*]]*/;
  Eigen::Matrix<double, 4, 1> box_k;
  box_k << /*[[*/ x1_max /*]*/,
      /*[*/ -x1_min /*]*/,
      /*[*/ x2_max /*]*/,
      /*[*/ -x2_min /*]]*/;
  ::aos::controls::HPolytope<2> t_poly(box_H, box_k);
  return ::aos::controls::HVPolytope<2, 4, 4>(t_poly.H(), t_poly.k(),
                                              t_poly.Vertices());
}

class CoerceGoalTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// WHOOOHH!
TEST_F(CoerceGoalTest, Inside) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << /*[[*/ 1, -1 /*]]*/;

  Eigen::Matrix<double, 2, 1> R;
  R << /*[[*/ 1.5, 1.5 /*]]*/;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(R(0, 0), output(0, 0));
  EXPECT_EQ(R(1, 0), output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_Intersect) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_no_Intersect) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(3, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(3.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Middle_Of_Edge) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(0, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << -1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, PerpendicularLine) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

// TODO(austin): Make sure the profile reset code when we disable works.

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
