package y2017.control_loops.serializer;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group SerializerQueue {
  implements aos.control_loops.ControlLoop;

  // All angles are in radians, and angular velocities are in radians/second.
  // For all angular velocities, positive is moving balls up towards the
  // shooter.

  message Goal {
    // Angular velocity goals in radians/second.
    double angular_velocity;
  };

  message Position {
    // Serializer angle in radians.
    double theta;
  };

  message Status {
    // The current average velocity in radians/second.
    double avg_angular_velocity;

    // The current instantaneous filtered velocity in radians/second.
    double angular_velocity;

    // True if the shooter is ready.  It is better to compare the velocities
    // directly so there isn't confusion on if the goal is up to date.
    bool ready;
  };

  message Output {
    // Voltage in volts of the shooter motor.
    double voltage;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group SerializerQueue shooter_queue;
