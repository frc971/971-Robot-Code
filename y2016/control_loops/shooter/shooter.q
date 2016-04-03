package y2016.control_loops.shooter;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct ShooterSideStatus {
  // True if the shooter side is up to speed and stable.
  bool ready;
  // The current average velocity in radians/second.
  double avg_angular_velocity;
  // The current instantaneous filtered velocity in radians/second.
  double angular_velocity;
};

queue_group ShooterQueue {
  implements aos.control_loops.ControlLoop;

  // All angles are in radians, and angular velocities are in radians/second.
  // For all angular velocities, positive is shooting the ball out of the robot.

  message Goal {
    // Angular velocity goals in radians/second.
    double angular_velocity;

    bool clamp_open; // True to release our clamp on the ball.
    // True to push the ball into the shooter.
    // If we are in the act of shooting with a goal velocity != 0, wait until it
    // is up to speed, push the ball into the shooter, and then wait until it
    // spins up and down before letting the piston be released.
    bool push_to_shooter;

    // Forces the lights on.
    bool force_lights_on;

    // If true, the robot is shooting forwards.
    bool shooting_forwards;
  };

  message Position {
    // Wheel angle in radians/second.
    double theta_left;
    double theta_right;
  };

  message Status {
    // Left side status.
    ShooterSideStatus left;
    // Right side status.
    ShooterSideStatus right;

    // True if the shooter is ready.  It is better to compare the velocities
    // directly so there isn't confusion on if the goal is up to date.
    bool ready;

    // The number of shots that have been fired since the start of the shooter
    // control loop.
    uint32_t shots;
  };

  message Output {
    // Voltage in volts of the left and right shooter motors.
    double voltage_left;
    double voltage_right;

    // See comments on the identical fields in Goal for details.
    bool clamp_open;
    bool push_to_shooter;

    // If true, the lights are on.
    bool lights_on;

    bool forwards_flashlight;
    bool backwards_flashlight;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ShooterQueue shooter_queue;
