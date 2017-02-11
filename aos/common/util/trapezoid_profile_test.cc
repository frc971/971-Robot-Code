#include "gtest/gtest.h"

#include "Eigen/Dense"

#include "aos/common/util/trapezoid_profile.h"

namespace aos {
namespace util {
namespace testing {

class TrapezoidProfileTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:
  TrapezoidProfileTest() : profile_(delta_time) {
    position_.setZero();
    profile_.set_maximum_acceleration(0.75);
    profile_.set_maximum_velocity(1.75);
  }

  // Runs an iteration.
  void RunIteration(double goal_position,
                    double goal_velocity) {
    position_ = profile_.Update(goal_position,
                                goal_velocity);
  }

  const Eigen::Matrix<double, 2, 1> &position() { return position_; }

  TrapezoidProfile profile_;

  ::testing::AssertionResult At(double position, double velocity) {
    static const double kDoubleNear = 0.00001;
    if (::std::abs(velocity - position_(1)) > kDoubleNear) {
      return ::testing::AssertionFailure() << "velocity is " << position_(1) <<
          " not " << velocity;
    }
    if (::std::abs(position - position_(0)) > kDoubleNear) {
      return ::testing::AssertionFailure() << "position is " << position_(0) <<
          " not " << position;
    }
    return ::testing::AssertionSuccess() << "at " << position <<
        " moving at " << velocity;
  }

 private:
  static constexpr ::std::chrono::nanoseconds delta_time =
      ::std::chrono::milliseconds(10);

  Eigen::Matrix<double, 2, 1> position_;
};

constexpr ::std::chrono::nanoseconds TrapezoidProfileTest::delta_time;

TEST_F(TrapezoidProfileTest, ReachesGoal) {
  for (int i = 0; i < 450; ++i) {
    RunIteration(3, 0);
  }
  EXPECT_TRUE(At(3, 0));
}

// Tests that decresing the maximum velocity in the middle when it is already
// moving faster than the new max is handled correctly.
TEST_F(TrapezoidProfileTest, ContinousUnderVelChange) {
  profile_.set_maximum_velocity(1.75);
  RunIteration(12.0, 0);
  double last_pos = position()(0);
  double last_vel = 1.75;
  for (int i = 0; i < 1600; ++i) {
    if (i == 400) {
      profile_.set_maximum_velocity(0.75);
    }
    RunIteration(12.0, 0);
    if (i >= 400) {
      EXPECT_TRUE(::std::abs(last_pos - position()(0)) <= 1.75 * 0.01);
      EXPECT_NEAR(last_vel, ::std::abs(last_pos - position()(0)), 0.0001);
    }
    last_vel = ::std::abs(last_pos - position()(0));
    last_pos = position()(0);
  }
  EXPECT_TRUE(At(12.0, 0));
}

// There is some somewhat tricky code for dealing with going backwards.
TEST_F(TrapezoidProfileTest, Backwards) {
  for (int i = 0; i < 400; ++i) {
    RunIteration(-2, 0);
  }
  EXPECT_TRUE(At(-2, 0));
}

TEST_F(TrapezoidProfileTest, SwitchGoalInMiddle) {
  for (int i = 0; i < 200; ++i) {
    RunIteration(-2, 0);
  }
  EXPECT_FALSE(At(-2, 0));
  for (int i = 0; i < 550; ++i) {
    RunIteration(0, 0);
  }
  EXPECT_TRUE(At(0, 0));
}

// Checks to make sure that it hits top speed.
TEST_F(TrapezoidProfileTest, TopSpeed) {
  for (int i = 0; i < 200; ++i) {
    RunIteration(4, 0);
  }
  EXPECT_NEAR(1.5, position()(1), 10e-5);
  for (int i = 0; i < 2000; ++i) {
    RunIteration(4, 0);
  }
  EXPECT_TRUE(At(4, 0));
}

// Tests that the position and velocity exactly match at the end.  Some code we
// have assumes this to be true as a simplification.
TEST_F(TrapezoidProfileTest, ExactlyReachesGoal) {
  for (int i = 0; i < 450; ++i) {
    RunIteration(1, 0);
  }
  EXPECT_EQ(position()(1), 0.0);
  EXPECT_EQ(position()(0), 1.0);
}

}  // namespace testing
}  // namespace util
}  // namespace aos
