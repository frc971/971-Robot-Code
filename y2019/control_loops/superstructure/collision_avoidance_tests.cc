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
    bool was_collided = avoidance.IsCollided(&status);

    while (true) {
      avoidance.UpdateGoal(&status, &unsafe_goal);
      if (!was_collided) {
        EXPECT_FALSE(avoidance.IsCollided(&status));
      } else {
        was_collided = avoidance.IsCollided(&status);
      }

      safe_goal.wrist.unsafe_goal =
          ::aos::Clip(unsafe_goal.wrist.unsafe_goal, avoidance.min_wrist_goal(),
                      avoidance.max_wrist_goal());

      safe_goal.elevator.unsafe_goal = ::std::max(unsafe_goal.elevator.unsafe_goal,
                                             avoidance.min_elevator_goal());

      safe_goal.intake.unsafe_goal =
          ::aos::Clip(unsafe_goal.intake.unsafe_goal,
                      avoidance.min_intake_goal(), avoidance.max_intake_goal());

      LimitedMove(&status.wrist.position, safe_goal.wrist.unsafe_goal);
      LimitedMove(&status.elevator.position, safe_goal.elevator.unsafe_goal);
      LimitedMove(&status.intake.position, safe_goal.intake.unsafe_goal);
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
    ASSERT_NEAR(unsafe_goal.wrist.unsafe_goal, status.wrist.position, 0.001);
    ASSERT_NEAR(unsafe_goal.elevator.unsafe_goal, status.elevator.position, 0.001);
    ASSERT_NEAR(unsafe_goal.intake.unsafe_goal, status.intake.position, 0.001);
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
  unsafe_goal.wrist.unsafe_goal = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
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
  unsafe_goal.wrist.unsafe_goal = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
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
  unsafe_goal.wrist.unsafe_goal = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
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
  unsafe_goal.wrist.unsafe_goal = avoidance.kWristMinAngle + avoidance.kEpsWrist;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
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
  unsafe_goal.wrist.unsafe_goal =
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0);
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
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
  unsafe_goal.wrist.unsafe_goal =
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0);
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
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
  unsafe_goal.wrist.unsafe_goal = 4.0;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status.wrist.position = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  status.elevator.position = 0.45;
  status.intake.position = avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  Iterate();

  ASSERT_NEAR(unsafe_goal.wrist.unsafe_goal, status.wrist.position, 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearWristDownHeight + avoidance.kEps),
              status.elevator.position, 0.001);
  ASSERT_NEAR(unsafe_goal.intake.unsafe_goal, status.intake.position, 0.001);
}

// Unreasonable Wrist Goal
TEST_F(CollisionAvoidanceTests, UnreasonableWristGoal) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal.wrist.unsafe_goal = avoidance.kWristMinAngle;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status.wrist.position = avoidance.kWristMaxAngle - avoidance.kEpsWrist;
  status.elevator.position = 0.45;
  status.intake.position =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  Iterate();

  ASSERT_NEAR(unsafe_goal.wrist.unsafe_goal, status.wrist.position, 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearIntakeHeight + avoidance.kEps),
              status.elevator.position, 0.001);
  ASSERT_NEAR(unsafe_goal.intake.unsafe_goal, status.intake.position, 0.001);
}

// Fix Collision Wrist in Elevator
TEST_F(CollisionAvoidanceTests, FixElevatorCollision) {
  // changes the goals
  unsafe_goal.wrist.unsafe_goal = 3.14;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  // sets the status position messgaes
  status.wrist.position = 4.0;
  status.elevator.position = 0.0;
  status.intake.position = avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  Iterate();

  ASSERT_NEAR(unsafe_goal.wrist.unsafe_goal, status.wrist.position, 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearWristDownHeight + avoidance.kEps),
              status.elevator.position, 0.001);
  ASSERT_NEAR(unsafe_goal.intake.unsafe_goal, status.intake.position, 0.001);
}

// Fix Collision Wrist in Intake
TEST_F(CollisionAvoidanceTests, FixWristCollision) {
  // changes the goals
  unsafe_goal.wrist.unsafe_goal = avoidance.kWristMinAngle + avoidance.kEpsWrist;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  // sets the status position messgaes
  status.wrist.position = avoidance.kWristMinAngle + avoidance.kEpsWrist;
  status.elevator.position = 0.0;
  status.intake.position =
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0;

  Iterate();

  ASSERT_NEAR(unsafe_goal.wrist.unsafe_goal, status.wrist.position, 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearIntakeHeight + avoidance.kEps),
              status.elevator.position, 0.001);
  ASSERT_NEAR(unsafe_goal.intake.unsafe_goal, status.intake.position, 0.001);
}
// Fix Collision Wrist Above Elevator
TEST_F(CollisionAvoidanceTests, FixWristElevatorCollision) {
  // changes the goals
  unsafe_goal.wrist.unsafe_goal = 0.0;
  unsafe_goal.elevator.unsafe_goal = 0.0;
  unsafe_goal.intake.unsafe_goal =
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  // sets the status position messgaes
  status.wrist.position = 0.0;
  status.elevator.position = 0.0;
  status.intake.position = avoidance.kIntakeOutAngle + avoidance.kEpsIntake;

  Iterate();

  ASSERT_NEAR(unsafe_goal.wrist.unsafe_goal, status.wrist.position, 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearHeight + avoidance.kEps),
              status.elevator.position, 0.001);
  ASSERT_NEAR(unsafe_goal.intake.unsafe_goal, status.intake.position, 0.001);
}
}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
