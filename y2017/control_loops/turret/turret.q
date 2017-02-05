package y2017.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct JointState {
  // Angle of the joint in radians.
  float angle;
  // Angular velocity of the joint in radians/second.
  float angular_velocity;
  // Profiled goal angle of the joint in radians.
  float goal_angle;
  // Profiled goal angular velocity of the joint in radians/second.
  float goal_angular_velocity;
  // Unprofiled goal angle of the joint in radians.
  float unprofiled_goal_angle;
  // Unprofiled goal angular velocity of the joint in radians/second.
  float unprofiled_goal_angular_velocity;

  // The estimated voltage error.
  float voltage_error;

  // The calculated velocity with delta x/delta t
  float calculated_velocity;

  // Components of the control loop output
  float position_power;
  float velocity_power;
  float feedforwards_power;

  // State of the estimator.
  .frc971.EstimatorState estimator_state;
};

queue_group TurretQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // An azimuth angle of zero means the turrent faces toward the front of the
    // robot where the intake is located. The angle increases when the turret
    // turns clockwise, and decreases when the turrent turns counter-clockwise.
    // These are from a top view above the robot.
    double angle_azimuth;

    // TODO(phil): Define how we control the vertical angle of the turret: is
    // it a distance or an angle, etc.
    double angle_altitude;

    // Caps on velocity/acceleration for profiling. 0 for the default.
    .frc971.ProfileParameters profile_params_turret;
  };

  message Status {
    // Is the turret zeroed?
    bool zeroed;

    // If true, we have aborted.
    bool estopped;

    // Estimate angles and angular velocities.
    JointState azimuth;
    JointState altitude;
  };

  message Position {
    // The sensor readings for the azimuth. The units and sign are defined the
    // same as what's in the Goal message.
    .frc971.PotAndAbsolutePosition azimuth;

    // TODO(phil): How are we measuring that other aspect of the turret (i.e.
    // the "altitude") ?
  };

  message Output {
    float voltage_azimuth;
    float voltage_altitude;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group TurretQueue turret_queue;
