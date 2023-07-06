#ifndef FRC971_CONTROL_LOOPS_AIMING_AIMING_H_
#define FRC971_CONTROL_LOOPS_AIMING_AIMING_H_
#include "frc971/constants.h"
#include "frc971/control_loops/pose.h"

// This library provides utilities associated with attempting to aim balls into
// a goal.

namespace frc971::control_loops::aiming {

// Control modes for managing how we manage shooting on the fly.
enum class ShotMode {
  // Don't do any shooting-on-the-fly compensation--just point straight at the
  // target. Primarily used in tests.
  kStatic,
  // Do do shooting-on-the-fly compensation.
  kShootOnTheFly,
};

struct TurretGoal {
  // Goal position (in radians) for the turret.
  double position = 0.0;
  // Goal velocity (in radians / sec) for the turret.
  double velocity = 0.0;
  // Physical distance from the robot's origin to the target we are shooting at,
  // in meters.
  double target_distance = 0.0;
  // Shot distance to use when shooting on the fly (e.g., if driving towards the
  // target, we will aim for a shorter shot than the actual physical distance),
  // in meters.
  double virtual_shot_distance = 0.0;
};

struct RobotState {
  // Pose of the robot, in the field frame.
  Pose pose;
  // X/Y components of the robot velocity, in m/s.
  Eigen::Vector2d velocity;
  // Yaw rate of the robot, in rad / sec.
  double yaw_rate;
  // Last turret goal that we produced.
  double last_turret_goal;
};

struct ShotConfig {
  // Pose of the goal, in the field frame.
  Pose goal;
  ShotMode mode;
  const constants::Range turret_range;
  // We assume that the ball being shot has an ~constant speed over the ground,
  // to allow us to estimate shooting-on-the fly values.
  double ball_speed_over_ground;
  // Amount of buffer to add on each side of the range to prevent wrapping/to
  // prevent getting too close to the hard stops.
  double anti_wrap_buffer;
  // Offset from zero in the robot frame to zero for the turret.
  double turret_zero_offset;
};

TurretGoal AimerGoal(const ShotConfig &config, const RobotState &state);
}  // namespace frc971::control_loops::aiming
#endif  // FRC971_CONTROL_LOOPS_AIMING_AIMING_H_
