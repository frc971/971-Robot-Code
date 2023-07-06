#include "y2022/control_loops/superstructure/collision_avoidance.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/commonmath.h"
#include "aos/flatbuffers.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

namespace y2022::control_loops::superstructure::testing {

using aos::FlatbufferDetachedBuffer;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

FlatbufferDetachedBuffer<Goal> MakeZeroGoal() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_front_offset;
  {
    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder intake_front_builder(
        fbb);

    intake_front_builder.add_unsafe_goal(0.0);
    intake_front_offset = intake_front_builder.Finish();
  }

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_back_offset;
  {
    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder intake_back_builder(
        fbb);

    intake_back_builder.add_unsafe_goal(0.0);
    intake_back_offset = intake_back_builder.Finish();
  }

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      turret_offset;
  {
    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder turret_builder(fbb);

    turret_builder.add_unsafe_goal(0.0);
    turret_offset = turret_builder.Finish();
  }

  superstructure::Goal::Builder goal_builder(fbb);

  goal_builder.add_intake_front(intake_front_offset);
  goal_builder.add_intake_back(intake_back_offset);
  goal_builder.add_turret(turret_offset);

  fbb.Finish(goal_builder.Finish());
  return fbb.Release();
}

// Enums for the different classes of intake and turret states
enum class IntakeState { kSafe, kUnsafe };
enum class TurretState {
  kSafeFront,
  kSafeBack,
  kSafeFrontWrapped,
  kSafeBackWrapped,
  kUnsafeFront,
  kUnsafeBack,
  kUnsafeFrontWrapped,
  kUnsafeBackWrapped,
  kNegativeSafeFront,
  kNegativeSafeBack,
  kNegativeSafeFrontWrapped,
  kNegativeSafeBackWrapped,
  kNegativeUnsafeFront,
  kNegativeUnsafeBack,
  kNegativeUnsafeFrontWrapped,
  kNegativeUnsafeBackWrapped,
};
enum class CatapultState { kIdle, kShooting };

class CollisionAvoidanceTest : public ::testing::Test {
 public:
  CollisionAvoidanceTest()
      : unsafe_goal_(MakeZeroGoal()),
        status_({0.0, 0.0, 0.0, false}),
        prev_status_({0.0, 0.0, 0.0, false}) {}

  void Simulate() {
    FlatbufferDetachedBuffer<Goal> safe_goal = MakeZeroGoal();
    bool was_collided = avoidance_.IsCollided(status_);

    bool moving = true;
    while (moving) {
      // Compute the safe goal
      avoidance_.UpdateGoal(status_, unsafe_goal_.message().turret());

      if (!was_collided) {
        // The system should never be collided if it didn't start off collided
        EXPECT_FALSE(avoidance_.IsCollided(status_));
      } else {
        was_collided = avoidance_.IsCollided(status_);
      }

      safe_goal.mutable_message()->mutable_intake_front()->mutate_unsafe_goal(
          ::aos::Clip(intake_front_goal(), avoidance_.min_intake_front_goal(),
                      avoidance_.max_intake_front_goal()));
      safe_goal.mutable_message()->mutable_intake_back()->mutate_unsafe_goal(
          ::aos::Clip(intake_back_goal(), avoidance_.min_intake_back_goal(),
                      avoidance_.max_intake_back_goal()));
      safe_goal.mutable_message()->mutable_turret()->mutate_unsafe_goal(
          ::aos::Clip(turret_goal(), avoidance_.min_turret_goal(),
                      avoidance_.max_turret_goal()));

      // Move each subsystem towards their goals a bit
      status_.intake_front_position =
          LimitedMove(status_.intake_front_position,
                      safe_goal.message().intake_front()->unsafe_goal());
      status_.intake_back_position =
          LimitedMove(status_.intake_back_position,
                      safe_goal.message().intake_back()->unsafe_goal());
      status_.turret_position = LimitedMove(
          status_.turret_position, safe_goal.message().turret()->unsafe_goal());

      // If it stopped moving, we're done
      if (!IsMoving()) {
        moving = false;
      } else {
        prev_status_ = status_;
      }
    }

    EXPECT_FALSE(avoidance_.IsCollided(status_));

    CheckGoals();
  }

  bool IsMoving() { return (status_ != prev_status_); }

  double ComputeIntakeAngle(IntakeState intake_state) {
    double intake_angle = 0.0;

    switch (intake_state) {
      case IntakeState::kSafe:
        intake_angle = CollisionAvoidance::kCollisionZoneIntake -
                       CollisionAvoidance::kEpsIntake;
        break;
      case IntakeState::kUnsafe:
        intake_angle = CollisionAvoidance::kCollisionZoneIntake +
                       CollisionAvoidance::kEpsIntake;
        break;
    }

    return intake_angle;
  }

  double ComputeTurretAngle(TurretState turret_state) {
    double turret_angle = 0.0;

    constexpr double kMaxCollisionZoneFrontTurretWrapped =
        (2.0 * M_PI) + CollisionAvoidance::kMaxCollisionZoneFrontTurret;
    constexpr double kMaxCollisionZoneBackTurretWrapped =
        (2.0 * M_PI) + CollisionAvoidance::kMaxCollisionZoneBackTurret;

    switch (turret_state) {
      case TurretState::kSafeFront:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneFrontTurret +
                       CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kSafeBack:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneBackTurret +
                       CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kSafeFrontWrapped:
        turret_angle = kMaxCollisionZoneFrontTurretWrapped +
                       CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kSafeBackWrapped:
        turret_angle =
            kMaxCollisionZoneBackTurretWrapped + CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kUnsafeFront:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneFrontTurret -
                       CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kUnsafeBack:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneBackTurret -
                       CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kUnsafeFrontWrapped:
        turret_angle = kMaxCollisionZoneFrontTurretWrapped -
                       CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kUnsafeBackWrapped:
        turret_angle =
            kMaxCollisionZoneBackTurretWrapped - CollisionAvoidance::kEpsTurret;
        break;

      case TurretState::kNegativeSafeFront:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneFrontTurret +
                       CollisionAvoidance::kEpsTurret - 2.0 * M_PI;
        break;
      case TurretState::kNegativeSafeBack:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneBackTurret +
                       CollisionAvoidance::kEpsTurret - 2.0 * M_PI;
        break;
      case TurretState::kNegativeSafeFrontWrapped:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneFrontTurret +
                       CollisionAvoidance::kEpsTurret - 4.0 * M_PI;
        break;
      case TurretState::kNegativeSafeBackWrapped:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneBackTurret +
                       CollisionAvoidance::kEpsTurret - 4.0 * M_PI;
        break;
      case TurretState::kNegativeUnsafeFront:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneFrontTurret -
                       CollisionAvoidance::kEpsTurret - 2.0 * M_PI;
        break;
      case TurretState::kNegativeUnsafeBack:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneBackTurret -
                       CollisionAvoidance::kEpsTurret - 2.0 * M_PI;
        break;
      case TurretState::kNegativeUnsafeFrontWrapped:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneFrontTurret -
                       CollisionAvoidance::kEpsTurret - 4.0 * M_PI;
        break;
      case TurretState::kNegativeUnsafeBackWrapped:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneBackTurret -
                       CollisionAvoidance::kEpsTurret - 4.0 * M_PI;
        break;
    }

    return turret_angle;
  }

  void Test(IntakeState intake_front_pos_state,
            IntakeState intake_back_pos_state, TurretState turret_pos_state,
            IntakeState intake_front_goal_state,
            IntakeState intake_back_goal_state, TurretState turret_goal_state,
            CatapultState catapult_state) {
    status_ = {ComputeIntakeAngle(intake_front_pos_state),
               ComputeIntakeAngle(intake_back_pos_state),
               ComputeTurretAngle(turret_pos_state),
               catapult_state == CatapultState::kShooting};

    unsafe_goal_.mutable_message()->mutable_intake_front()->mutate_unsafe_goal(
        ComputeIntakeAngle(intake_front_goal_state));
    unsafe_goal_.mutable_message()->mutable_intake_back()->mutate_unsafe_goal(
        ComputeIntakeAngle(intake_back_goal_state));

    unsafe_goal_.mutable_message()->mutable_turret()->mutate_unsafe_goal(
        ComputeTurretAngle(turret_goal_state));

    Simulate();
  }

  // Provide goals and status messages
  FlatbufferDetachedBuffer<Goal> unsafe_goal_;
  CollisionAvoidance::Status status_;

 private:
  static constexpr double kIterationMove = 0.001;

  void CheckGoals() {
    // Check to see if we reached the goals
    // Turret is highest priority and should always reach the unsafe goal
    EXPECT_NEAR(turret_goal(), status_.turret_position, kIterationMove);

    // If the unsafe goal had an intake colliding with the turret or catapult,
    // the intake position should be at least the collision zone angle.
    // Otherwise, the intake should be at the unsafe goal
    if (avoidance_.TurretCollided(
            intake_front_goal(), turret_goal(),
            CollisionAvoidance::kMinCollisionZoneFrontTurret,
            CollisionAvoidance::kMaxCollisionZoneFrontTurret) ||
        (status_.shooting &&
         avoidance_.TurretCollided(
             intake_front_goal(), turret_goal() + M_PI,
             CollisionAvoidance::kMinCollisionZoneFrontTurret,
             CollisionAvoidance::kMaxCollisionZoneFrontTurret))) {
      EXPECT_LE(status_.intake_front_position,
                CollisionAvoidance::kCollisionZoneIntake);
    } else {
      EXPECT_NEAR(intake_front_goal(), status_.intake_front_position,
                  kIterationMove);
    }

    if (avoidance_.TurretCollided(
            intake_back_goal(), turret_goal(),
            CollisionAvoidance::kMinCollisionZoneBackTurret,
            CollisionAvoidance::kMaxCollisionZoneBackTurret) ||
        (status_.shooting &&
         avoidance_.TurretCollided(
             intake_back_goal(), turret_goal() + M_PI,
             CollisionAvoidance::kMinCollisionZoneBackTurret,
             CollisionAvoidance::kMaxCollisionZoneBackTurret))) {
      EXPECT_LE(status_.intake_back_position,
                CollisionAvoidance::kCollisionZoneIntake);
    } else {
      EXPECT_NEAR(intake_back_goal(), status_.intake_back_position, 0.001);
    }
  }

  double LimitedMove(double position, double goal) {
    if (position + kIterationMove < goal) {
      return position + kIterationMove;
    } else if (position - kIterationMove > goal) {
      return position - kIterationMove;
    } else {
      return goal;
    }
  }

  double intake_front_goal() const {
    return unsafe_goal_.message().intake_front()->unsafe_goal();
  }
  double intake_back_goal() const {
    return unsafe_goal_.message().intake_back()->unsafe_goal();
  }
  double turret_goal() const {
    return unsafe_goal_.message().turret()->unsafe_goal();
  }

  CollisionAvoidance avoidance_;
  CollisionAvoidance::Status prev_status_;
};

// Just to be safe, brute force ALL the possible position-goal combinations
// and make sure we never collide and the correct goals are reached
TEST_F(CollisionAvoidanceTest, BruteForce) {
  // Intake front position
  for (IntakeState intake_front_pos :
       {IntakeState::kSafe, IntakeState::kUnsafe}) {
    // Intake back position
    for (IntakeState intake_back_pos :
         {IntakeState::kSafe, IntakeState::kUnsafe}) {
      // Turret back position
      for (TurretState turret_pos :
           {TurretState::kSafeFront, TurretState::kSafeBack,
            TurretState::kSafeFrontWrapped, TurretState::kSafeBackWrapped,
            TurretState::kUnsafeFront, TurretState::kUnsafeBack,
            TurretState::kUnsafeFrontWrapped, TurretState::kUnsafeBackWrapped,
            TurretState::kNegativeSafeFront, TurretState::kNegativeSafeBack,
            TurretState::kNegativeSafeFrontWrapped,
            TurretState::kNegativeSafeBackWrapped,
            TurretState::kNegativeUnsafeFront, TurretState::kNegativeUnsafeBack,
            TurretState::kNegativeUnsafeFrontWrapped,
            TurretState::kNegativeUnsafeBackWrapped}) {
        // Intake front goal
        for (IntakeState intake_front_goal :
             {IntakeState::kSafe, IntakeState::kUnsafe}) {
          // Intake back goal
          for (IntakeState intake_back_goal :
               {IntakeState::kSafe, IntakeState::kUnsafe}) {
            // Turret goal
            for (TurretState turret_goal :
                 {TurretState::kSafeFront, TurretState::kSafeBack,
                  TurretState::kSafeFrontWrapped, TurretState::kSafeBackWrapped,
                  TurretState::kUnsafeFront, TurretState::kUnsafeBack,
                  TurretState::kUnsafeFrontWrapped,
                  TurretState::kUnsafeBackWrapped,
                  TurretState::kNegativeSafeFront,
                  TurretState::kNegativeSafeBack,
                  TurretState::kNegativeSafeFrontWrapped,
                  TurretState::kNegativeSafeBackWrapped,
                  TurretState::kNegativeUnsafeFront,
                  TurretState::kNegativeUnsafeBack,
                  TurretState::kNegativeUnsafeFrontWrapped,
                  TurretState::kNegativeUnsafeBackWrapped}) {
              // Catapult state
              for (CatapultState catapult_state :
                   {CatapultState::kIdle, CatapultState::kShooting}) {
                Test(intake_front_pos, intake_back_pos, turret_pos,
                     intake_front_goal, intake_back_goal, turret_goal,
                     catapult_state);
              }
            }
          }
        }
      }
    }
  }
}

// Tests some of the various wraps to make sure they match what we expect.
TEST(WrapTest, Wrap) {
  EXPECT_THAT(WrapTurretAngle(0.0), ::testing::Eq(std::make_pair(0.0, 0)));

  EXPECT_THAT(WrapTurretAngle(M_PI * 2.0 - 0.1),
              ::testing::Pair(::testing::DoubleNear(M_PI * 2.0 - 0.1, 1e-6),
                              ::testing::Eq(0)));

  EXPECT_THAT(
      WrapTurretAngle(M_PI * 2.0 + 0.1),
      ::testing::Pair(::testing::DoubleNear(0.1, 1e-6), ::testing::Eq(1)));

  EXPECT_THAT(WrapTurretAngle(M_PI * 4.0 - 0.1),
              ::testing::Pair(::testing::DoubleNear(M_PI * 2.0 - 0.1, 1e-6),
                              ::testing::Eq(1)));

  EXPECT_THAT(
      WrapTurretAngle(M_PI * 4.0 + 0.1),
      ::testing::Pair(::testing::DoubleNear(0.1, 1e-6), ::testing::Eq(2)));

  EXPECT_THAT(
      WrapTurretAngle(-M_PI * 2.0 + 0.1),
      ::testing::Pair(::testing::DoubleNear(0.1, 1e-6), ::testing::Eq(-1)));
  EXPECT_THAT(WrapTurretAngle(-0.1),
              ::testing::Pair(::testing::DoubleNear(M_PI * 2.0 - 0.1, 1e-6),
                              ::testing::Eq(-1)));

  EXPECT_THAT(
      WrapTurretAngle(-M_PI * 4.0 + 0.1),
      ::testing::Pair(::testing::DoubleNear(0.1, 1e-6), ::testing::Eq(-2)));
  EXPECT_THAT(WrapTurretAngle(-M_PI * 2.0 - 0.1),
              ::testing::Pair(::testing::DoubleNear(M_PI * 2.0 - 0.1, 1e-6),
                              ::testing::Eq(-2)));

  EXPECT_THAT(
      WrapTurretAngle(-M_PI * 6.0 + 0.1),
      ::testing::Pair(::testing::DoubleNear(0.1, 1e-6), ::testing::Eq(-3)));
  EXPECT_THAT(WrapTurretAngle(-M_PI * 4.0 - 0.1),
              ::testing::Pair(::testing::DoubleNear(M_PI * 2.0 - 0.1, 1e-6),
                              ::testing::Eq(-3)));
}

// Tests that wrap -> unwrap is a nop.
TEST(WrapTest, UnWrap) {
  int wraps = -10000;
  double wrapped = 0.0;
  for (double i = -50; i < 50; i += 0.01) {
    std::pair<double, int> r = WrapTurretAngle(i);

    // Do a dummy check.  The wraps should always increase since the angle is
    // increasing, the wrapped angle too, and everything should be within a
    // decent range.
    EXPECT_GE(r.first, 0.0);
    EXPECT_LT(r.first, 2.0 * M_PI);
    if (wraps != r.second) {
      EXPECT_GT(r.second, wraps);
      wraps = r.second;
    } else {
      EXPECT_GT(r.first, wrapped);
    }
    wrapped = r.first;

    // And unwrapping should get us back to the start.
    EXPECT_THAT(UnwrapTurretAngle(r.first, r.second),
                ::testing::DoubleNear(i, 1e-9));
  }
}

// Test that AngleInRange works correctly for wrapped angles
TEST(AngleTest, AngleInRange) {
  EXPECT_TRUE(AngleInRange(0.5, 0.4, 0.6));
  EXPECT_TRUE(AngleInRange(0, (2.0 * M_PI) - 0.2, 0.2));
  EXPECT_FALSE(AngleInRange(0, (2.0 * M_PI) - 0.2, (2.0 * M_PI) - 0.1));
  EXPECT_TRUE(AngleInRange(0.5, (2.0 * M_PI) - 0.1, 0.6));
}

}  // namespace y2022::control_loops::superstructure::testing
