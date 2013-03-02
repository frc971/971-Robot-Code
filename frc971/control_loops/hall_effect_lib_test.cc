#include <array>

#include <memory>

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/queue.h"
#include "aos/common/queue_testutils.h"
#include "gtest/gtest.h"
#include "frc971/control_loops/hall_effect_loop.h"
#include "frc971/control_loops/hall_effect_loop-inl.h"

namespace frc971 {
namespace control_loops {
namespace testing {

// Functions creating a StateFeedbackLoop stuff borrowing constants from wrist.
// The constants are not very important, they just need to be reasonable.
StateFeedbackPlant<2, 1, 1> MakeHallEffectPlant() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00876530955899, 0.0, 0.763669024671;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.000423500644841, 0.0810618735867;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlant<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackLoop<2, 1, 1> MakeHallEffectLoop() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.66366902467, 58.1140316091;
  Eigen::Matrix<double, 1, 2> K;
  K << 31.5808145893, 0.867171288023;
  return StateFeedbackLoop<2, 1, 1>(L, K, MakeHallEffectPlant());
}

class HallEffectSimulation : public ::testing::Test {
 public:
  ::aos::common::testing::GlobalCoreInstance my_core;
  HallEffectSimulation() 
    : hall_effect_loop_(new StateFeedbackLoop<2, 1, 1>(MakeHallEffectLoop()),
        true, 5.0),
    hall_effect_plant_(new StateFeedbackPlant<2, 1, 1>(MakeHallEffectPlant())),
    lower_limit_(0.0),
    upper_limit_(2.0) {
    hall_stop_[0] = 0.5;
    hall_stop_[1] = 1.25;
    hall_stop_[2] = 2.0;
    hall_start_[0] = -0.1;
    hall_start_[1] = 0.75;
    hall_start_[2] = 1.5;
    // Flush the robot state queue.
    ::aos::robot_state.Clear();
    SendDSPacket(true);
  }

  // Sends needed messages to the queue so that the zeroing code knows
  // if the robot is enabled or not.
  void SendDSPacket(bool enabled) {
    ::aos::robot_state.MakeWithBuilder().enabled(enabled)
                                        .autonomous(false)
                                        .team_id(971).Send();
    ::aos::robot_state.FetchLatest();
  }

  // Resets the hall_effect_loop_ state to uninitialized, sets the position
  // of the simulation to initial_position and sets the zero_down value of
  // the hall_effect_loop.
  void Reinitialize(double initial_position, bool zero_down) {
    initial_position_ = initial_position;
    hall_effect_loop_.zero_down_ = zero_down;
    hall_effect_plant_->X(0, 0) = initial_position_;
    hall_effect_plant_->X(1, 0) = 0.0;
    hall_effect_plant_->Y =
      hall_effect_plant_->C * hall_effect_plant_->X;
    last_position_ = hall_effect_plant_->Y(0, 0);
    calibration_[0] = -initial_position;
    calibration_[1] = -initial_position;
    calibration_[2] = -initial_position;
  }
  
  // Returns the absolute angle (change since last Reinitialize).
  double GetAbsolutePosition() const {
    return hall_effect_plant_->Y(0, 0);
  }

  // Returns adjusted angle (the "real" angle).
  double GetPosition() const {
    return GetAbsolutePosition() - initial_position_;
  }

  // Resets the state feedback loop to a certain position and sets the
  // HallEffectLoop to either zero up or down.
  // good_position refers to whether or not the position from the queue
  // would have been good or not.
  // Simulates the system for one timestep.
  void Simulate(double goal, bool good_position) {
    last_position_ = GetAbsolutePosition();
    for (int i = 0; i < 3; ++i) {
      if (last_position_ > hall_start_[i] && last_position_ < hall_stop_[i]) {
        hall_effect_[i] = true;
      } else {
        hall_effect_[i] = false;
      }
    }
    if (hall_effect_loop_.zero_down_) {
      hall_effect_loop_.UpdateZeros(hall_stop_, hall_effect_, calibration_,
          1.0, GetPosition(), good_position);
    } else {
      hall_effect_loop_.UpdateZeros(hall_start_, hall_effect_, calibration_,
          1.0, GetPosition(), good_position);
    }
    if (hall_effect_loop_.state_ == HallEffectLoop<3>::READY) {
      hall_effect_loop_.loop_->R(0, 0) = goal;
      hall_effect_loop_.loop_->R(1, 0) = 0.0;
    }
    // Deal with updating all the necessary observers and the such.
    hall_effect_loop_.loop_->Update(true, false);
    hall_effect_plant_->U << hall_effect_loop_.loop_->U(0, 0);
    hall_effect_plant_->Update();
  }

  void VerifyNearGoal(double goal) {
    EXPECT_NEAR(goal, GetAbsolutePosition(), 1e-4);
  }

  virtual ~HallEffectSimulation() {
    ::aos::robot_state.Clear();
  }

  HallEffectLoop<3> hall_effect_loop_;
  ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> hall_effect_plant_;
 private:
  // Physical limits (normally in constants.cpp).
  double lower_limit_;
  double upper_limit_;
  // Edges of the hall effect sensors (normally in constants.cpp).
  ::std::array<double, 3> hall_start_;
  ::std::array<double, 3> hall_stop_;
  // Array of whether any given hall effect sensor is activated.
  ::std::array<bool, 3> hall_effect_;
  // Array of calibration values (normally from queue).
  ::std::array<double, 3> calibration_;
  double initial_position_;
  double last_position_;
  double calibration_value_[3];
};  // class HallEffectSimulation

// Just test normal zeroing, and then going to a goal.
TEST_F(HallEffectSimulation, ZerosCorrectly) {
  Reinitialize(1.3, true);
  for (int i = 0; i < 200; ++i) {
    SendDSPacket(true);
    Simulate(0.55, true);
  }
  VerifyNearGoal(0.55);
}

// Tests whether it zeros correctly starting on a sensor.
TEST_F(HallEffectSimulation, ZeroStartingOn) {
  Reinitialize(1.0, true);
  for (int i = 0; i < 200; ++i) {
    SendDSPacket(true);
    Simulate(0.55, true);
  }
  VerifyNearGoal(0.55);
}

// Tests non-zero_down behavior (zero_up).
TEST_F(HallEffectSimulation, ZeroUp) {
  Reinitialize(1.4, false);
  for (int i = 0; i < 200; ++i) {
    SendDSPacket(true);
    Simulate(0.55, true);
  }
  VerifyNearGoal(0.55);
}

// Tests dealing with missing positions.
TEST_F(HallEffectSimulation, ZerosUp) {
  Reinitialize(1.4, true);
  for (int i = 0; i < 5; ++i) {
    SendDSPacket(true);
    Simulate(0.55, true);
  }
  for (int i = 0; i < 50; ++i) {
    SendDSPacket(true);
    Simulate(0.55, false);
  }
  EXPECT_TRUE(hall_effect_loop_.state_ != HallEffectLoop<3>::READY);
  for (int i = 0; i < 150; ++i) {
    SendDSPacket(true);
    Simulate(0.55, true);
  }
  VerifyNearGoal(0.55);
}

// Tests whether the zeroing goal really does get limitted if it gets too high.
TEST_F(HallEffectSimulation, LimitsZeroingGoal) {
  Reinitialize(.6, true);
  SendDSPacket(true);
  Simulate(.1, true);
  hall_effect_loop_.zeroing_position_ = -30;
  hall_effect_loop_.loop_->R(0, 0) = -30;
  hall_effect_loop_.loop_->Update(true, false);
  hall_effect_loop_.LimitZeroingGoal();
  EXPECT_NEAR(hall_effect_loop_.zeroing_position_, 0, .6);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
