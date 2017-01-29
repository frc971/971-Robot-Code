package y2017.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct JointState {
  // Distance of the joint, in m, from absolute zero.
  float distance;
  // Linear velocity of the joint in meters/second.
  float linear_velocity;
  // Profiled goal distance of the joint in meters.
  float goal_distance;
  // Profiled goal linear velocity of the joint in meters/second.
  float goal_linear_velocity;
  // Unprofiled goal distance from absoulte zero  of the joint in meters.
  float unprofiled_goal_distance;
  // Unprofiled goal linear velocity of the joint in meters/second.
  float unprofiled_goal_linear_velocity;

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

queue_group IntakeQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Zero on the intake is when the intake is retracted inside the robot,
    // unable to intake. Positive is out.

    // Goal distance of the intake.
    double distance_intake;

    // Caps on velocity/acceleration for profiling. 0 for the default.
    .frc971.ProfileParameters profile_params_intake;

    // Voltage to send to the rollers. Positive is sucking in.
    float voltage_rollers;
  };

  message Status {
    // Is the intake zeroed?
    bool zeroed;

    // If true, we have aborted.
    bool estopped;

    // Estimate angles and angular velocities.
    JointState intake;
  };

  message Position {
    // Position of the intake, zero when the intake is in, positive when it is
    // out.
    .frc971.PotAndAbsolutePosition intake;
  };

  message Output {
    float voltage_intake;

    float voltage_rollers;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group IntakeQueue intake_queue;
