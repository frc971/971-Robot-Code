#include "y2019/control_loops/superstructure/collision_avoidance.h"

#include "aos/commonmath.h"
#include "gtest/gtest.h"
#include "y2019/control_loops/superstructure/superstructure.q.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {
namespace testing {

/*
Test List:
    FullClockwiseRotationFromBottomBackIntakeIn
    QuarterClockwiseRotationFromMiddleFrontIntakeOut
    QuarterClockwiseRotationFromMiddleFrontIntakeMiddle
    QuarterClockwiseRotationFromMiddleFrontIntakeMoving

    FullCounterClockwiseRotationFromBottomFrontIntakeIn
    QuarterCounterClockwiseRotationFromBottomFrontIntakeOut
    QuarterCounterClockwiseRotationFromBottomFrontIntakeMiddle
    QuarterCounterClockwiseRotationFromBottomFrontIntakeMoving
*/

class CollisionAvoidanceTests : public ::testing::Test {
 public:
  void Iterate() {
    SuperstructureQueue::Goal safe_goal;
    while (true) {
      avoidance.UpdateGoal(&status, &unsafe_goal);

      EXPECT_FALSE(avoidance.IsCollided(&status));
      safe_goal.wrist.angle =
          ::aos::Clip(unsafe_goal.wrist.angle, avoidance.min_wrist_goal(),
                      avoidance.max_wrist_goal());

      safe_goal.elevator.height = ::std::max(unsafe_goal.elevator.height,
                                             avoidance.min_elevator_goal());

      safe_goal.intake.joint_angle =
          ::aos::Clip(unsafe_goal.intake.joint_angle,
                      avoidance.min_intake_goal(), avoidance.max_intake_goal());

      LimitedMove(&status.wrist.position, safe_goal.wrist.angle);
      LimitedMove(&status.elevator.position, safe_goal.elevator.height);
      LimitedMove(&status.intake.position, safe_goal.intake.joint_angle);
      if (IsMoving()) {
        break;
      }
      past_status = status;
    }
  }

  bool IsMoving() {
    if ((past_status.wrist.position == status.wrist.position) &&
        (past_status.elevator.position == status.elevator.position) &&
        (past_status.intake.position == status.intake.position)) {
      return true;
    } else {
      return false;
    }
  }

  // provide goals and status messages
  SuperstructureQueue::Goal unsafe_goal;
  SuperstructureQueue::Status status;
  SuperstructureQueue::Status past_status;

 protected:
  // setup for all tests
  CollisionAvoidance avoidance;

  void CheckGoals() {
    // check to see if we reached the goals
    ASSERT_NEAR(unsafe_goal.wrist.angle, status.wrist.position, 0.001);
    ASSERT_NEAR(unsafe_goal.elevator.height, status.elevator.position, 0.001);
    ASSERT_NEAR(unsafe_goal.intake.joint_angle, status.intake.position, 0.001);
  }

 private:
  void LimitedMove(float *position, double goal) {
    if (*position + kIterationMove < goal) {
      *position += kIterationMove;
    } else if (*position - kIterationMove > goal) {
      *position -= kIterationMove;
    } else {
      *position = goal;
    }
  }

  static constexpr double kIterationMove = 0.001;
};

// It is trying to rotate from far back to front low.
TEST_F(CollisionAvoidanceTests, FullClockwiseRotationFromBottomBackIntakeIn) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // in.
  unsafe_goal.wrist.angle = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      avoidance.kIntakeInAngle - avoidance.kEpsIntake;

  // sets the status position messgaes to be have the elevator at the bottom
  // with the intake in and the wrist low back
  status.wrist.position = avoidance.kWristMinAngle + avoidance.kEpsWrist;
  status.elevator.position = 0.0;
  status.intake.position = avoidance.kIntakeInAngle - avoidance.kEpsIntake;

  Iterate();

  CheckGoals();
}

// It is trying to rotate from the front middle to front bottom.
TEST_F(CollisionAvoidanceTests,
       QuarterClockwiseRotationFromMiddleFrontIntakeOut) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal.wrist.angle = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status.wrist.position =
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0);
  status.elevator.position = 0.0;
  status.intake.position = avoidance.kIntakeOutAngle;

  Iterate();

  CheckGoals();
}
// It is trying to rotate from the front middle to front bottom.
TEST_F(CollisionAvoidanceTests,
       QuarterClockwiseRotationFromMiddleFrontIntakeMiddle) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // in.
  status.wrist.position = avoidance.kWristMaxAngle / 2.0;
  status.elevator.position = 0.5;
  status.intake.position =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  unsafe_goal.wrist.angle = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  Iterate();

  CheckGoals();
}

// It is trying to rotate from front low to far back.
TEST_F(CollisionAvoidanceTests,
       FullCounterClockwiseRotationFromBottomBackIntakeIn) {
  // sets the status position messgaes to be have the elevator at the bottom
  // with the intake in and the wrist low back
  status.wrist.position = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  status.elevator.position = 0.0;
  status.intake.position = avoidance.kIntakeInAngle - avoidance.kEpsIntake;

  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // in.
  unsafe_goal.wrist.angle = avoidance.kWristMinAngle + avoidance.kEpsWrist;
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      avoidance.kIntakeInAngle - avoidance.kEpsIntake;

  Iterate();

  CheckGoals();
}

// It is trying to rotate from the front bottom to front middle.
TEST_F(CollisionAvoidanceTests,
       QuarterCounterClockwiseRotationFromMiddleFrontIntakeOut) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal.wrist.angle =
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0);
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status.wrist.position = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  status.elevator.position = 0.0;
  status.intake.position = avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  Iterate();

  CheckGoals();
}

// It is trying to rotate from the front bottom to front middle.
TEST_F(CollisionAvoidanceTests,
       QuarterCounterClockwiseRotationFromBottomFrontIntakeMiddle) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal.wrist.angle =
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0);
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status.wrist.position = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  status.elevator.position = 0.5;
  status.intake.position =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  Iterate();

  CheckGoals();
}

// Unreasonable Elevator Goal
TEST_F(CollisionAvoidanceTests, UnreasonableElevatorGoal) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal.wrist.angle = 4.0;
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status.wrist.position = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  status.elevator.position = 0.45;
  status.intake.position = avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  Iterate();

  ASSERT_NEAR(unsafe_goal.wrist.angle, status.wrist.position, 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearWristDownHeight + avoidance.kEps),
              status.elevator.position, 0.001);
  ASSERT_NEAR(unsafe_goal.intake.joint_angle, status.intake.position, 0.001);
}

// Unreasonable Wrist Goal
TEST_F(CollisionAvoidanceTests, UnreasonableWristGoal) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal.wrist.angle = avoidance.kWristMinAngle;
  unsafe_goal.elevator.height = 0.0;
  unsafe_goal.intake.joint_angle =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status.wrist.position = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  status.elevator.position = 0.45;
  status.intake.position =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  Iterate();

  ASSERT_NEAR(unsafe_goal.wrist.angle, status.wrist.position, 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearIntakeHeight + avoidance.kEps),
              status.elevator.position, 0.001);
  ASSERT_NEAR(unsafe_goal.intake.joint_angle, status.intake.position, 0.001);
}

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
