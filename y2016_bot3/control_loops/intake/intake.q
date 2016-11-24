package y2016_bot3.control_loops;

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

queue_group IntakeQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Zero on the intake is when the horizontal tube stock members are level
    // with the ground.  This will be essentially when we are in the intaking
    // position.  Positive is up.  The angle is measured relative to the top
    // of the robot frame.
    // Zero on the shoulder is horizontal.  Positive is up.  The angle is
    // measured relative to the top of the robot frame.
    // Zero on the wrist is horizontal and landed in the bellypan.  Positive is
    // the same direction as the shoulder.  The angle is measured relative to
    // the top of the robot frame.

    // Goal angles and angular velocities of the intake.
    double angle_intake;

    // Caps on velocity/acceleration for profiling. 0 for the default.
    float max_angular_velocity_intake;

    float max_angular_acceleration_intake;

    // Voltage to send to the rollers. Positive is sucking in.
    float voltage_top_rollers;
    float voltage_bottom_rollers;
    float voltage_intake_rollers;

    bool force_intake;

    // If true, fire the traverse mechanism down.
    bool traverse_down;
  };

  message Status {
    // Is the intake zeroed?
    bool zeroed;

    // If true, we have aborted.
    bool estopped;

    // The internal state of the state machine.
    int32_t state;

    // Estimated angle and angular velocitie of the intake.
    JointState intake;
  };

  message Position {
    // Zero for the intake potentiometer value is horizontal, and positive is
    // up.
    .frc971.PotAndIndexPosition intake;
  };

  message Output {
    float voltage_intake;

    float voltage_top_rollers;
    float voltage_bottom_rollers;
    float voltage_intake_rollers;

    // If true, fire the traverse mechanism down.
    bool traverse_down;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group IntakeQueue intake_queue;
