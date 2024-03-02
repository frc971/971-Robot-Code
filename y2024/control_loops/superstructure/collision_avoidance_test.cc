#include "y2024/control_loops/superstructure/collision_avoidance.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/commonmath.h"
#include "aos/flatbuffers.h"
#include "y2024/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2024/control_loops/superstructure/superstructure_status_generated.h"

namespace y2024::control_loops::superstructure::testing {

using aos::FlatbufferDetachedBuffer;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

// Enums for the different classes of intake and turret states
enum class IntakeState { kSafe, kUnsafe };
enum class TurretState {
  kSafe,
  kUnsafe,
};
enum class ExtendState {
  kSafe,
  kUnsafe,
};

class CollisionAvoidanceTest : public ::testing::Test {
 public:
  CollisionAvoidanceTest()
      : intake_goal_(0),
        turret_goal_(0),
        extend_goal_(0),
        status_({0.0, 0.0, 0.0}),
        prev_status_({0.0, 0.0, 0.0}) {}

  void Simulate() {
    double safe_intake_goal = 0;
    double safe_turret_goal = 0;
    double safe_extend_goal = 0;
    bool was_collided = avoidance_.IsCollided(status_);

    VLOG(1) << "Simulation of intake " << status_.intake_pivot_position
            << ", intake_goal " << intake_goal_ << ", turret "
            << status_.turret_position << ", turret_goal " << turret_goal_
            << ", extend " << status_.extend_position << ", extend goal "
            << extend_goal_;

    bool moving = true;
    while (moving) {
      // Compute the safe goal
      avoidance_.UpdateGoal(status_, turret_goal_, extend_goal_);

      if (!was_collided) {
        // The system should never be collided if it didn't start off collided
        EXPECT_FALSE(avoidance_.IsCollided(status_))
            << ": Now collided at intake " << status_.intake_pivot_position
            << ", intake_goal " << intake_goal_ << ", turret "
            << status_.turret_position << ", turret_goal " << turret_goal_
            << ", extend " << status_.extend_position << ", extend goal "
            << extend_goal_;
      } else {
        was_collided = avoidance_.IsCollided(status_);
      }

      safe_intake_goal =
          ::aos::Clip(intake_goal_, avoidance_.min_intake_pivot_goal(),
                      avoidance_.max_intake_pivot_goal());

      safe_turret_goal = ::aos::Clip(turret_goal_, avoidance_.min_turret_goal(),
                                     avoidance_.max_turret_goal());

      safe_extend_goal = ::aos::Clip(extend_goal_, avoidance_.min_extend_goal(),
                                     avoidance_.max_extend_goal());

      // Move each subsystem towards their goals a bit
      status_.intake_pivot_position =
          LimitedMove(status_.intake_pivot_position, safe_intake_goal);
      status_.turret_position =
          LimitedMove(status_.turret_position, safe_turret_goal);
      status_.extend_position =
          LimitedMove(status_.extend_position, safe_extend_goal);

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

    switch (turret_state) {
      case TurretState::kSafe:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneTurret +
                       CollisionAvoidance::kEpsTurret;
        break;
      case TurretState::kUnsafe:
        turret_angle = CollisionAvoidance::kMaxCollisionZoneTurret -
                       CollisionAvoidance::kEpsTurret;
        break;
    }

    return turret_angle;
  }

  double ComputeExtendPosition(ExtendState extend_state) {
    double extend_position = 0.0;

    switch (extend_state) {
      case ExtendState::kSafe:
        extend_position = CollisionAvoidance::kMinCollisionZoneExtend -
                          CollisionAvoidance::kEpsExtend;
        break;
      case ExtendState::kUnsafe:
        extend_position = CollisionAvoidance::kMinCollisionZoneExtend +
                          CollisionAvoidance::kEpsTurret;
        break;
    }

    return extend_position;
  }

  void Test(IntakeState intake_front_pos_state, TurretState turret_pos_state,
            ExtendState extend_pos_state, IntakeState intake_front_goal_state,
            TurretState turret_goal_state, ExtendState extend_goal_state) {
    status_ = {ComputeIntakeAngle(intake_front_pos_state),
               ComputeTurretAngle(turret_pos_state),
               ComputeExtendPosition(extend_pos_state)};

    intake_goal_ = ComputeIntakeAngle(intake_front_goal_state);

    turret_goal_ = ComputeTurretAngle(turret_goal_state);

    extend_goal_ = ComputeExtendPosition(extend_goal_state);

    Simulate();
  }

  // Provide goals and status messages
  double intake_goal_;
  double turret_goal_;
  double extend_goal_;
  CollisionAvoidance::Status status_;

 private:
  static constexpr double kIterationMove = 0.001;

  void CheckGoals() {
    // Check to see if we reached the goals
    // Turret is highest priority and should always reach the unsafe goal
    EXPECT_NEAR(extend_goal_, status_.extend_position, kIterationMove);

    if (avoidance_.ExtendCollided(intake_goal_, turret_goal_, extend_goal_)) {
      EXPECT_EQ(status_.turret_position,
                CollisionAvoidance::kSafeTurretExtendedPosition);
      // If the unsafe goal had an intake colliding with the turret the intake
      // position should be at least the collision zone angle. Otherwise, the
      // intake should be at the unsafe goal
      if (avoidance_.TurretCollided(intake_goal_, status_.turret_position,
                                    extend_goal_)) {
        EXPECT_LE(status_.intake_pivot_position,
                  CollisionAvoidance::kCollisionZoneIntake);
      } else {
        EXPECT_NEAR(intake_goal_, status_.intake_pivot_position,
                    kIterationMove);
      }
    } else {
      EXPECT_NEAR(turret_goal_, status_.turret_position, kIterationMove);
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

  CollisionAvoidance avoidance_;
  CollisionAvoidance::Status prev_status_;
};  // namespace y2024::control_loops::superstructure::testing

// Just to be safe, brute force ALL the possible position-goal combinations
// and make sure we never collide and the correct goals are reached
TEST_F(CollisionAvoidanceTest, BruteForce) {
  // Intake front position
  for (IntakeState intake_front_pos :
       {IntakeState::kSafe, IntakeState::kUnsafe}) {
    // Turret back position
    for (TurretState turret_pos : {TurretState::kSafe, TurretState::kUnsafe}) {
      for (ExtendState extend_pos :
           {ExtendState::kSafe, ExtendState::kUnsafe}) {
        // Intake front goal
        for (IntakeState intake_front_goal :
             {IntakeState::kSafe, IntakeState::kUnsafe}) {
          // Turret goal
          for (TurretState turret_goal :
               {TurretState::kSafe, TurretState::kUnsafe}) {
            for (ExtendState extend_goal :
                 {ExtendState::kSafe, ExtendState::kUnsafe}) {
              Test(intake_front_pos, turret_pos, extend_pos, intake_front_goal,
                   turret_goal, extend_goal);
            }
          }
        }
      }
    }
  }
}

}  // namespace y2024::control_loops::superstructure::testing
