#include "aos/util/trapezoid_profile.h"

#include <compare>
#include <cstdlib>
#include <memory>
#include <ratio>

#include "Eigen/Dense"  // IWYU pragma: keep
#include "gtest/gtest.h"

namespace aos::util::testing {

class TrapezoidProfileTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:
  TrapezoidProfileTest() : profile_(kDeltaTime) {
    position_.setZero();
    profile_.set_maximum_acceleration(0.75);
    profile_.set_maximum_deceleration(0.75);
    profile_.set_maximum_velocity(1.75);
  }

  // Runs an iteration.
  void RunIteration(double goal_position, double goal_velocity) {
    position_ = profile_.Update(goal_position, goal_velocity);
  }

  void RunFor(double goal_position, double goal_velocity,
              std::chrono::nanoseconds duration) {
    while (duration > std::chrono::nanoseconds(0)) {
      position_ = profile_.Update(goal_position, goal_velocity);
      duration -= kDeltaTime;
    }

    ASSERT_EQ(duration.count(), 0);
  }

  const Eigen::Matrix<double, 2, 1> &position() { return position_; }

  AsymmetricTrapezoidProfile profile_;

  ::testing::AssertionResult At(double position, double velocity) {
    static const double kDoubleNear = 0.00001;
    if (::std::abs(velocity - position_(1)) > kDoubleNear) {
      return ::testing::AssertionFailure()
             << "velocity is " << position_(1) << " not " << velocity;
    }
    if (::std::abs(position - position_(0)) > kDoubleNear) {
      return ::testing::AssertionFailure()
             << "position is " << position_(0) << " not " << position;
    }
    return ::testing::AssertionSuccess()
           << "at " << position << " moving at " << velocity;
  }

 private:
  static constexpr ::std::chrono::nanoseconds kDeltaTime =
      ::std::chrono::milliseconds(10);

  Eigen::Matrix<double, 2, 1> position_;
};

constexpr ::std::chrono::nanoseconds TrapezoidProfileTest::kDeltaTime;

TEST_F(TrapezoidProfileTest, ReachesGoal) {
  RunFor(3, 0, std::chrono::milliseconds(4500));
  EXPECT_TRUE(At(3, 0));
}

// Tests that decreasing the maximum velocity in the middle when it is already
// moving faster than the new max is handled correctly.
TEST_F(TrapezoidProfileTest, ContinousUnderVelChange) {
  profile_.set_maximum_velocity(1.75);
  RunFor(12.0, 0, std::chrono::milliseconds(10));
  double last_pos = position()(0);
  double last_vel = 1.75;
  for (int i = 0; i < 1600; ++i) {
    if (i == 400) {
      profile_.set_maximum_velocity(0.75);
    }
    RunFor(12.0, 0, std::chrono::milliseconds(10));
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
  RunFor(-2, 0, std::chrono::milliseconds(4000));
  EXPECT_TRUE(At(-2, 0));
}

TEST_F(TrapezoidProfileTest, SwitchGoalInMiddle) {
  RunFor(-2, 0, std::chrono::milliseconds(2000));
  EXPECT_FALSE(At(-2, 0));
  RunFor(0, 0, std::chrono::milliseconds(5500));
  EXPECT_TRUE(At(0, 0));
}

// Checks to make sure that it hits top speed.
TEST_F(TrapezoidProfileTest, TopSpeed) {
  RunFor(4, 0, std::chrono::milliseconds(2000));
  EXPECT_NEAR(1.5, position()(1), 10e-5);
  RunFor(4, 0, std::chrono::milliseconds(20000));
  EXPECT_TRUE(At(4, 0));
}

// Tests that the position and velocity exactly match at the end.  Some code we
// have assumes this to be true as a simplification.
TEST_F(TrapezoidProfileTest, ExactlyReachesGoal) {
  RunFor(1, 0, std::chrono::milliseconds(4500));
  EXPECT_EQ(position()(1), 0.0);
  EXPECT_EQ(position()(0), 1.0);
}

// Tests that we can move a goal without the trajectory teleporting.  The goal
// needs to move to something we haven't already passed, but will blow by.
TEST_F(TrapezoidProfileTest, MoveGoal) {
  profile_.set_maximum_acceleration(2.0);
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(2.0);

  RunFor(5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(1.0, 2.0));
  RunFor(5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(3.0, 2.0));
  RunFor(3.5, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(4.0, 0.0));
  RunFor(3.5, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(3.5, 0.0));
}

// Tests that we can move a goal back before where we currently are without
// teleporting.
TEST_F(TrapezoidProfileTest, MoveGoalFar) {
  profile_.set_maximum_acceleration(2.0);
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(2.0);

  RunFor(5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(1.0, 2.0));
  RunFor(5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(3.0, 2.0));
  RunFor(2.5, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(4.0, 0.0));
  RunFor(2.5, 0, std::chrono::seconds(2));
  EXPECT_TRUE(At(2.5, 0.0));
}

// Tests that we can move a goal without the trajectory teleporting.  The goal
// needs to move to something we haven't already passed, but will blow by.  Do
// this one in the negative direction.
TEST_F(TrapezoidProfileTest, MoveGoalNegative) {
  profile_.set_maximum_acceleration(2.0);
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(2.0);

  RunFor(-5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-1.0, -2.0));
  RunFor(-5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-3.0, -2.0));
  RunFor(-3.5, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-4.0, 0.0));
  RunFor(-3.5, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-3.5, 0.0));
}

// Tests that we can move a goal back before where we currently are without
// teleporting.  Do this one in the negative direction.
TEST_F(TrapezoidProfileTest, MoveGoalNegativeFar) {
  profile_.set_maximum_acceleration(2.0);
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(2.0);

  RunFor(-5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-1.0, -2.0));
  RunFor(-5.0, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-3.0, -2.0));
  RunFor(-2.5, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-4.0, 0.0));
  RunFor(-2.5, 0, std::chrono::seconds(2));
  EXPECT_TRUE(At(-2.5, 0.0));
}

// Tests that we can execute a profile with acceleration and deceleration not
// matching in magnitude.
TEST_F(TrapezoidProfileTest, AsymmetricAccelDecel) {
  // Accelerates up until t=1.  Will be at x=0.5
  profile_.set_maximum_acceleration(1.0);
  // Decelerates in t=0.5  Will take x=0.25
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(1.0);

  RunFor(1.75, 0, std::chrono::seconds(1));

  EXPECT_TRUE(At(0.5, 1.0));

  RunFor(1.75, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(1.5, 1.0));
  RunFor(1.75, 0, std::chrono::milliseconds(500));
  EXPECT_TRUE(At(1.75, 0.0));
}

// Tests that we can execute a profile with acceleration and deceleration not
// matching in magnitude, and hitting saturation.
TEST_F(TrapezoidProfileTest, AsymmetricAccelDecelUnconstrained) {
  // Accelerates up until t=1.  Will be at x=0.5
  profile_.set_maximum_acceleration(1.0);
  // Decelerates in t=0.5  Will take x=0.25
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(2.0);

  RunFor(0.75, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(0.5, 1.0));

  RunFor(0.75, 0, std::chrono::milliseconds(500));
  EXPECT_TRUE(At(0.75, 0.0));
}

// Tests that we can execute a profile with acceleration and deceleration not
// matching in magnitude, and hitting saturation.
TEST_F(TrapezoidProfileTest, AsymmetricAccelDecelUnconstrainedNegative) {
  // Accelerates up until t=1.  Will be at x=0.5
  profile_.set_maximum_acceleration(1.0);
  // Decelerates in t=0.5  Will take x=0.25
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(2.0);

  RunFor(-0.75, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-0.5, -1.0));

  RunFor(-0.75, 0, std::chrono::milliseconds(500));
  EXPECT_TRUE(At(-0.75, 0.0));
}

// Tests that we can execute a profile with acceleration and deceleration not
// matching in magnitude when going in the negative direction.
TEST_F(TrapezoidProfileTest, AsymmetricAccelDecelNegative) {
  // Accelerates up until t=1.  Will be at x=0.5
  profile_.set_maximum_acceleration(1.0);
  // Decelerates in t=0.5  Will take x=0.25
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(1.0);

  RunFor(-1.75, 0, std::chrono::seconds(1));

  EXPECT_TRUE(At(-0.5, -1.0));

  RunFor(-1.75, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(-1.5, -1.0));
  RunFor(-1.75, 0, std::chrono::milliseconds(500));
  EXPECT_TRUE(At(-1.75, 0.0));
}

// Tests that we can move the goal when an asymmetric profile is executing.
TEST_F(TrapezoidProfileTest, AsymmetricAccelDecelMoveGoal) {
  // Accelerates up until t=1.  Will be at x=0.5
  profile_.set_maximum_acceleration(1.0);
  // Decelerates in t=0.5  Will take x=0.25
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(1.0);

  RunFor(1.75, 0, std::chrono::seconds(1));

  EXPECT_TRUE(At(0.5, 1.0));

  RunFor(1.75, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(1.5, 1.0));
  RunFor(1.6, 0, std::chrono::milliseconds(500));
  EXPECT_TRUE(At(1.75, 0.0));
  RunFor(1.6, 0, std::chrono::milliseconds(520));
  RunFor(1.6, 0, std::chrono::milliseconds(2500));
  EXPECT_TRUE(At(1.6, 0.0));
}

// Tests that we can move the goal when an asymmetric profile is executing in
// the negative direction.
TEST_F(TrapezoidProfileTest, AsymmetricAccelDecelMoveGoalFar) {
  // Accelerates up until t=1.  Will be at x=0.5
  profile_.set_maximum_acceleration(1.0);
  // Decelerates in t=0.5  Will take x=0.25
  profile_.set_maximum_deceleration(2.0);
  profile_.set_maximum_velocity(1.0);

  RunFor(1.75, 0, std::chrono::seconds(1));

  EXPECT_TRUE(At(0.5, 1.0));

  RunFor(1.75, 0, std::chrono::seconds(1));
  EXPECT_TRUE(At(1.5, 1.0));
  RunFor(1.0, 0, std::chrono::milliseconds(500));
  EXPECT_TRUE(At(1.75, 0.0));
  RunFor(1.0, 0, std::chrono::milliseconds(2500));
  EXPECT_TRUE(At(1.0, 0.0));
}

}  // namespace aos::util::testing
