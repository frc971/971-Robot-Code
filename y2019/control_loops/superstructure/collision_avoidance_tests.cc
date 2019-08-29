#include "y2019/control_loops/superstructure/collision_avoidance.h"

#include "aos/commonmath.h"
#include "aos/flatbuffers.h"
#include "gtest/gtest.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_status_generated.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {
namespace testing {

using aos::FlatbufferDetachedBuffer;
using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

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

FlatbufferDetachedBuffer<Goal> MakeZeroGoal() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal> wrist_offset;
  {
    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder wrist_builder(fbb);

    wrist_builder.add_unsafe_goal(0.0);
    wrist_offset = wrist_builder.Finish();
  }

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      elevator_offset;
  {
    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder elevator_builder(fbb);

    elevator_builder.add_unsafe_goal(0.0);
    elevator_offset = elevator_builder.Finish();
  }

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_offset;
  {
    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder intake_builder(fbb);

    intake_builder.add_unsafe_goal(0.0);
    intake_offset = intake_builder.Finish();
  }

  superstructure::Goal::Builder goal_builder(fbb);

  goal_builder.add_wrist(wrist_offset);
  goal_builder.add_elevator(elevator_offset);
  goal_builder.add_intake(intake_offset);

  fbb.Finish(goal_builder.Finish());
  return fbb.Release();
}

FlatbufferDetachedBuffer<Status> MakeZeroStatus() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus> wrist_offset;
  {
    PotAndAbsoluteEncoderProfiledJointStatus::Builder wrist_builder(fbb);

    wrist_builder.add_position(0.0);
    wrist_offset = wrist_builder.Finish();
  }

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      elevator_offset;
  {
    PotAndAbsoluteEncoderProfiledJointStatus::Builder elevator_builder(fbb);

    elevator_builder.add_position(0.0);
    elevator_offset = elevator_builder.Finish();
  }

  flatbuffers::Offset<AbsoluteEncoderProfiledJointStatus>
      intake_offset;
  {
    AbsoluteEncoderProfiledJointStatus::Builder intake_builder(fbb);

    intake_builder.add_position(0.0);
    intake_offset = intake_builder.Finish();
  }

  superstructure::Status::Builder goal_builder(fbb);

  goal_builder.add_wrist(wrist_offset);
  goal_builder.add_elevator(elevator_offset);
  goal_builder.add_intake(intake_offset);
  goal_builder.add_has_piece(false);

  fbb.Finish(goal_builder.Finish());
  return fbb.Release();
}

class CollisionAvoidanceTests : public ::testing::Test,
                                public ::testing::WithParamInterface<bool> {
 public:
  CollisionAvoidanceTests()
      : unsafe_goal_(MakeZeroGoal()), status_(MakeZeroStatus()) {
    status_.mutable_message()->mutate_has_piece(GetParam());
  }

  void Iterate() {
    FlatbufferDetachedBuffer<Goal> safe_goal = MakeZeroGoal();
    bool was_collided = avoidance.IsCollided(&status_.message());

    while (true) {
      avoidance.UpdateGoal(&status_.message(), &unsafe_goal_.message());
      if (!was_collided) {
        EXPECT_FALSE(avoidance.IsCollided(&status_.message()));
      } else {
        was_collided = avoidance.IsCollided(&status_.message());
      }

      safe_goal.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
          ::aos::Clip(unsafe_goal_.message().wrist()->unsafe_goal(),
                      avoidance.min_wrist_goal(), avoidance.max_wrist_goal()));

      safe_goal.mutable_message()->mutable_elevator()->mutate_unsafe_goal(
          ::std::max(unsafe_goal_.message().elevator()->unsafe_goal(),
                     avoidance.min_elevator_goal()));

      safe_goal.mutable_message()->mutable_intake()->mutate_unsafe_goal(
          ::aos::Clip(unsafe_goal_.message().intake()->unsafe_goal(),
                      avoidance.min_intake_goal(),
                      avoidance.max_intake_goal()));

      status_.mutable_message()->mutable_wrist()->mutate_position(
          LimitedMove(status_.message().wrist()->position(),
                      safe_goal.message().wrist()->unsafe_goal()));
      status_.mutable_message()->mutable_elevator()->mutate_position(
          LimitedMove(status_.message().elevator()->position(),
                      safe_goal.message().elevator()->unsafe_goal()));
      status_.mutable_message()->mutable_intake()->mutate_position(
          LimitedMove(status_.message().intake()->position(),
                      safe_goal.message().intake()->unsafe_goal()));
      if (IsMoving()) {
        break;
      }
      past_wrist_position_ = status_.message().wrist()->position();
      past_elevator_position_ = status_.message().elevator()->position();
      past_intake_position_ = status_.message().intake()->position();
    }
  }

  bool IsMoving() {
    if ((past_wrist_position_ == status_.message().wrist()->position()) &&
        (past_elevator_position_ == status_.message().elevator()->position()) &&
        (past_intake_position_ == status_.message().intake()->position())) {
      return true;
    } else {
      return false;
    }
  }

  // provide goals and status messages
  FlatbufferDetachedBuffer<Goal> unsafe_goal_;
  FlatbufferDetachedBuffer<Status> status_;
  float past_wrist_position_ = 0;
  float past_elevator_position_ = 0;
  float past_intake_position_ = 0;

 protected:
  // setup for all tests
  CollisionAvoidance avoidance;

  void CheckGoals() {
    // check to see if we reached the goals
    ASSERT_NEAR(unsafe_goal_.message().wrist()->unsafe_goal(),
                status_.message().wrist()->position(), 0.001);
    ASSERT_NEAR(unsafe_goal_.message().elevator()->unsafe_goal(),
                status_.message().elevator()->position(), 0.001);
    ASSERT_NEAR(unsafe_goal_.message().intake()->unsafe_goal(),
                status_.message().intake()->position(), 0.001);
  }

 private:
  float LimitedMove(float position, double goal) {
    if (position + kIterationMove < goal) {
      return position + kIterationMove;
    } else if (position - kIterationMove > goal) {
      return position - kIterationMove;
    } else {
      return goal;
    }
  }

  static constexpr double kIterationMove = 0.001;
};
// It is trying to rotate from far back to front low.
TEST_P(CollisionAvoidanceTests, FullClockwiseRotationFromBottomBackIntakeIn) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // in.
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeInAngle - avoidance.kEpsIntake);

  // sets the status position messgaes to be have the elevator at the bottom
  // with the intake in and the wrist low back
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMinAngle + avoidance.kEpsWrist);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.0);
  status_.mutable_message()->mutable_intake()->mutate_position(
      avoidance.kIntakeInAngle - avoidance.kEpsIntake);

  Iterate();

  CheckGoals();
}

// It is trying to rotate from the front middle to front bottom.
TEST_P(CollisionAvoidanceTests,
       QuarterClockwiseRotationFromMiddleFrontIntakeOut) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0));
  status_.mutable_message()->mutable_elevator()->mutate_position(0.0);
  status_.mutable_message()->mutable_intake()->mutate_position(
      avoidance.kIntakeOutAngle);

  Iterate();

  CheckGoals();
}
// It is trying to rotate from the front middle to front bottom.
TEST_P(CollisionAvoidanceTests,
       QuarterClockwiseRotationFromMiddleFrontIntakeMiddle) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // in.
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMaxAngle / 2.0);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.5);
  status_.mutable_message()->mutable_intake()->mutate_position(
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0);

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  Iterate();

  CheckGoals();
}

// It is trying to rotate from front low to far back.
TEST_P(CollisionAvoidanceTests,
       FullCounterClockwiseRotationFromBottomBackIntakeIn) {
  // sets the status position messgaes to be have the elevator at the bottom
  // with the intake in and the wrist low back
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.0);
  status_.mutable_message()->mutable_intake()->mutate_position(
      avoidance.kIntakeInAngle - avoidance.kEpsIntake);

  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // in.
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMinAngle + avoidance.kEpsWrist);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeInAngle - avoidance.kEpsIntake);

  Iterate();

  CheckGoals();
}

// It is trying to rotate from the front bottom to front middle.
TEST_P(CollisionAvoidanceTests,
       QuarterCounterClockwiseRotationFromMiddleFrontIntakeOut) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0));
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.0);
  status_.mutable_message()->mutable_intake()->mutate_position(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  Iterate();

  CheckGoals();
}

// It is trying to rotate from the front bottom to front middle.
TEST_P(CollisionAvoidanceTests,
       QuarterCounterClockwiseRotationFromBottomFrontIntakeMiddle) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMaxAngle - (avoidance.kEpsWrist * 2.0));
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.5);
  status_.mutable_message()->mutable_intake()->mutate_position(
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0);

  Iterate();

  CheckGoals();
}

// Unreasonable Elevator Goal
TEST_P(CollisionAvoidanceTests, UnreasonableElevatorGoal) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(4.0);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.45);
  status_.mutable_message()->mutable_intake()->mutate_position(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  Iterate();

  ASSERT_NEAR(unsafe_goal_.message().wrist()->unsafe_goal(),
              status_.message().wrist()->position(), 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearWristDownHeight + avoidance.kEps),
              status_.message().elevator()->position(), 0.001);
  ASSERT_NEAR(unsafe_goal_.message().intake()->unsafe_goal(),
              status_.message().intake()->position(), 0.001);
}

// Unreasonable Wrist Goal
TEST_P(CollisionAvoidanceTests, UnreasonableWristGoal) {
  // changes the goals to be in the position where the angle is low front and
  // the elevator is all the way at the bottom with the intake attempting to be
  // out.
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0);

  // sets the status position messgaes to be have the elevator at the half way
  // with the intake in and the wrist middle front
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMinAngle + avoidance.kEpsWrist);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.45);
  status_.mutable_message()->mutable_intake()->mutate_position(
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0);

  Iterate();

  EXPECT_NEAR(unsafe_goal_.message().wrist()->unsafe_goal(),
              status_.message().wrist()->position(), 0.001);
  EXPECT_NEAR((avoidance.kElevatorClearIntakeHeight + avoidance.kEps),
              status_.message().elevator()->position(), 0.001);
  EXPECT_NEAR(unsafe_goal_.message().intake()->unsafe_goal(),
              status_.message().intake()->position(), 0.001);
}

// Fix Collision Wrist in Elevator
TEST_P(CollisionAvoidanceTests, FixElevatorCollision) {
  // changes the goals
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(3.14);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  // sets the status position messgaes
  status_.mutable_message()->mutable_wrist()->mutate_position(4.0);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.0);
  status_.mutable_message()->mutable_intake()->mutate_position(
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  Iterate();

  ASSERT_NEAR(unsafe_goal_.message().wrist()->unsafe_goal(),
              status_.message().wrist()->position(), 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearWristDownHeight + avoidance.kEps),
              status_.message().elevator()->position(), 0.001);
  ASSERT_NEAR(unsafe_goal_.message().intake()->unsafe_goal(),
              status_.message().intake()->position(), 0.001);
}

// Fix Collision Wrist in Intake
TEST_P(CollisionAvoidanceTests, FixWristCollision) {
  // changes the goals
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal(0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal(
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0);

  // sets the status position messgaes
  status_.mutable_message()->mutable_wrist()->mutate_position(
      avoidance.kWristMaxAngle - avoidance.kEpsWrist);
  status_.mutable_message()->mutable_elevator()->mutate_position(0.0);
  status_.mutable_message()->mutable_intake()->mutate_position(
      (avoidance.kIntakeOutAngle + avoidance.kIntakeInAngle) / 2.0);

  Iterate();

  ASSERT_NEAR(unsafe_goal_.message().wrist()->unsafe_goal(),
              status_.message().wrist()->position(), 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearIntakeHeight + avoidance.kEps),
              status_.message().elevator()->position(), 0.001);
  ASSERT_NEAR(unsafe_goal_.message().intake()->unsafe_goal(),
              status_.message().intake()->position(), 0.001);
}
// Fix Collision Wrist Above Elevator
TEST_P(CollisionAvoidanceTests, FixWristElevatorCollision) {
  // changes the goals
  unsafe_goal_.mutable_message()->mutable_wrist()->mutate_unsafe_goal ( 0.0);
  unsafe_goal_.mutable_message()->mutable_elevator()->mutate_unsafe_goal ( 0.0);
  unsafe_goal_.mutable_message()->mutable_intake()->mutate_unsafe_goal (
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  // sets the status position messgaes
  status_.mutable_message()->mutable_wrist()->mutate_position ( 0.0);
  status_.mutable_message()->mutable_elevator()->mutate_position ( 0.0);
  status_.mutable_message()->mutable_intake()->mutate_position (
      avoidance.kIntakeOutAngle + avoidance.kEpsIntake);

  Iterate();

  ASSERT_NEAR(unsafe_goal_.message().wrist()->unsafe_goal(),
              status_.message().wrist()->position(), 0.001);
  ASSERT_NEAR((avoidance.kElevatorClearHeight + avoidance.kEps),
              status_.message().elevator()->position(), 0.001);
  ASSERT_NEAR(unsafe_goal_.message().intake()->unsafe_goal(),
              status_.message().intake()->position(), 0.001);
}

INSTANTIATE_TEST_CASE_P(CollisionAvoidancePieceTest, CollisionAvoidanceTests,
                        ::testing::Bool());

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
