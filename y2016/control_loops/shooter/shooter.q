package y2016.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group ShooterQueue {
  implements aos.control_loops.ControlLoop;

  // All angles are in radians, and angular velocities are in rad/sec. For all
  // angular velocities, positive is shooting the ball out of the robot.

  message Goal {
    // Angular velocity goals in rad/sec.
    double angular_velocity_left;
    double angular_velocity_right;
  };

  message Position {
    // Wheel angle in rad/sec.
    double theta_left;
    double theta_right;
  };

  message Status {
    bool ready_left;
    bool ready_right;
    // Is the shooter ready to shoot?
    bool ready_both;

    // Average angular velocities over dt * kHistoryLength.
    double avg_angular_velocity_left;
    double avg_angular_velocity_right;
  };

  message Output {
    double voltage_left;
    double voltage_right;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ShooterQueue shooter_queue;
