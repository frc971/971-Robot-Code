#include <unistd.h>

#include <memory>

#include "gtest/gtest.h"
#include "aos/common/network/team_number.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "aos/controls/polytope.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "frc971/queues/other_sensors.q.h"


using ::aos::time::Time;

namespace frc971 {
namespace control_loops {
namespace testing {

class Environment : public ::testing::Environment {
 public:
  virtual ~Environment() {}
  // how to set up the environment.
  virtual void SetUp() {
    aos::controls::HPolytope<0>::Init();
  }
};
::testing::Environment* const holder_env =
  ::testing::AddGlobalTestEnvironment(new Environment);

class TeamNumberEnvironment : public ::testing::Environment {
 public:
  // Override this to define how to set up the environment.
  virtual void SetUp() { aos::network::OverrideTeamNumber(971); }
};

::testing::Environment* const team_number_env =
    ::testing::AddGlobalTestEnvironment(new TeamNumberEnvironment);

// Class which simulates the drivetrain and sends out queue messages containing the
// position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation()
      : drivetrain_plant_(
            new StateFeedbackPlant<4, 2, 2>(MakeDrivetrainPlant())),
        my_drivetrain_loop_(".frc971.control_loops.drivetrain",
                       0x8a8dde77, ".frc971.control_loops.drivetrain.goal",
                       ".frc971.control_loops.drivetrain.position",
                       ".frc971.control_loops.drivetrain.output",
                       ".frc971.control_loops.drivetrain.status") {
    Reinitialize();
  }

  // Resets the plant.
  void Reinitialize() {
    drivetrain_plant_->X(0, 0) = 0.0;
    drivetrain_plant_->X(1, 0) = 0.0;
    drivetrain_plant_->Y = drivetrain_plant_->C() * drivetrain_plant_->X;
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
  }

  // Returns the position of the drivetrain.
  double GetLeftPosition() const {
    return drivetrain_plant_->Y(0, 0);
  }
  double GetRightPosition() const {
    return drivetrain_plant_->Y(1, 0);
  }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double left_encoder = GetLeftPosition();
    const double right_encoder = GetRightPosition();

    ::aos::ScopedMessagePtr<control_loops::Drivetrain::Position> position =
        my_drivetrain_loop_.position.MakeMessage();
    position->left_encoder = left_encoder;
    position->right_encoder = right_encoder;
    position.Send();
  }

  // Simulates the drivetrain moving for one timestep.
  void Simulate() {
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
    EXPECT_TRUE(my_drivetrain_loop_.output.FetchLatest());
    drivetrain_plant_->U << my_drivetrain_loop_.output->left_voltage,
                            my_drivetrain_loop_.output->right_voltage;
    drivetrain_plant_->Update();
  }

  ::std::unique_ptr<StateFeedbackPlant<4, 2, 2>> drivetrain_plant_;
 private:
  Drivetrain my_drivetrain_loop_;
  double last_left_position_;
  double last_right_position_;
};

class DrivetrainTest : public ::testing::Test {
 protected:
  ::aos::common::testing::GlobalCoreInstance my_core;

  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  Drivetrain my_drivetrain_loop_;

  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;

  DrivetrainTest() : my_drivetrain_loop_(".frc971.control_loops.drivetrain",
                               0x8a8dde77,
                               ".frc971.control_loops.drivetrain.goal",
                               ".frc971.control_loops.drivetrain.position",
                               ".frc971.control_loops.drivetrain.output",
                               ".frc971.control_loops.drivetrain.status"),
                drivetrain_motor_(&my_drivetrain_loop_),
                drivetrain_motor_plant_() {
    // Flush the robot state queue so we can use clean shared memory for this
    // test, also for the gyro.
    ::aos::robot_state.Clear();
    ::bbb::sensor_generation.Clear();
    ::bbb::sensor_generation.MakeWithBuilder()
        .reader_pid(254)
        .cape_resets(5)
        .Send();
    ::frc971::sensors::gyro_reading.Clear();
    SendDSPacket(true);
  }

  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled)
                                        .autonomous(false)
                                        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

  void VerifyNearGoal() {
    my_drivetrain_loop_.goal.FetchLatest();
    my_drivetrain_loop_.position.FetchLatest();
    EXPECT_NEAR(my_drivetrain_loop_.goal->left_goal,
                drivetrain_motor_plant_.GetLeftPosition(),
                1e-2);
    EXPECT_NEAR(my_drivetrain_loop_.goal->right_goal,
                drivetrain_motor_plant_.GetRightPosition(),
                1e-2);
  }

  virtual ~DrivetrainTest() {
    ::aos::robot_state.Clear();
    ::frc971::sensors::gyro_reading.Clear();
    ::bbb::sensor_generation.Clear();
  }
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  my_drivetrain_loop_.goal.MakeWithBuilder().control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0).Send();
  for (int i = 0; i < 200; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  my_drivetrain_loop_.goal.MakeWithBuilder().control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0).Send();
  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    if (i > 20 && i < 200) {
      SendDSPacket(false);
    } else {
      SendDSPacket(true);
    }
  }
  VerifyNearGoal();
}

// Tests surviving bad positions.
TEST_F(DrivetrainTest, SurvivesBadPosition) {
  my_drivetrain_loop_.goal.MakeWithBuilder().control_loop_driving(true)
      .left_goal(-1.0)
      .right_goal(1.0).Send();
  for (int i = 0; i < 500; ++i) {
    if (i > 20 && i < 200) {
    } else {
      drivetrain_motor_plant_.SendPositionMessage();
    }
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SendDSPacket(true);
  }
  VerifyNearGoal();
}

::aos::controls::HPolytope<2> MakeBox(double x1_min, double x1_max,
                                      double x2_min, double x2_max) {
  Eigen::Matrix<double, 4, 2> box_H;
  box_H << /*[[*/ 1.0, 0.0 /*]*/,
            /*[*/-1.0, 0.0 /*]*/,
            /*[*/ 0.0, 1.0 /*]*/,
            /*[*/ 0.0,-1.0 /*]]*/;
  Eigen::Matrix<double, 4, 1> box_k;
  box_k << /*[[*/ x1_max /*]*/,
            /*[*/-x1_min /*]*/,
            /*[*/ x2_max /*]*/,
            /*[*/-x2_min /*]]*/;
  ::aos::controls::HPolytope<2> t_poly(box_H, box_k);
  return t_poly;
}

class CoerceGoalTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// WHOOOHH!
TEST_F(CoerceGoalTest, Inside) {
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

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
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

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
  ::aos::controls::HPolytope<2> box = MakeBox(3, 4, 1, 2);

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
  ::aos::controls::HPolytope<2> box = MakeBox(0, 4, 1, 2);

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
  ::aos::controls::HPolytope<2> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
