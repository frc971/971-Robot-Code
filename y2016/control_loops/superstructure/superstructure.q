package y2016.control_loops;

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

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Zero on the intake is when the horizontal tube stock members are level
    // with the top frame rails of the robot.  This will be essentially when we
    // are in the intaking position.  Positive is up.  The angle is measured
    // relative to the top
    // of the robot frame.
    // Zero on the shoulder is when the shoulder is down against the hard stop
    // blocks.  Positive is up.  The angle is measured relative to the top of
    // the robot frame.
    // Zero on the wrist is horizontal and landed in the bellypan.  Positive is
    // the same direction as the shoulder.  The angle is measured relative to
    // the top of the robot frame.  For calibration, 0 is measured as parallel
    // to the big frame supporting the shooter.

    // Goal angles and angular velocities of the superstructure subsystems.
    double angle_intake;
    double angle_shoulder;
    // In relation to the ground plane.
    double angle_wrist;

    // Caps on velocity/acceleration for profiling. 0 for the default.
    float max_angular_velocity_intake;
    float max_angular_velocity_shoulder;
    float max_angular_velocity_wrist;

    float max_angular_acceleration_intake;
    float max_angular_acceleration_shoulder;
    float max_angular_acceleration_wrist;

    // Voltage to send to the rollers. Positive is sucking in.
    float voltage_top_rollers;
    float voltage_bottom_rollers;

    // Voltage to sent to the climber. Positive is pulling the robot up.
    float voltage_climber;
    // If true, unlatch the climber and allow it to unfold.
    bool unfold_climber;

    bool force_intake;

    // If true, release the latch which holds the traverse mechanism in the
    // middle.
    bool traverse_unlatched;
    // If true, fire the traverse mechanism down.
    bool traverse_down;
  };

  message Status {
    // Are the superstructure subsystems zeroed?
    bool zeroed;

    // If true, we have aborted.
    bool estopped;

    // The internal state of the state machine.
    int32_t state;


    // Estimated angles and angular velocities of the superstructure subsystems.
    JointState intake;
    JointState shoulder;
    JointState wrist;

    int32_t shoulder_controller_index;

    // Is the superstructure collided?
    bool is_collided;
  };

  message Position {
    // Zero for the intake potentiometer value is horizontal, and positive is
    // up.
    // Zero for the shoulder potentiometer value is horizontal, and positive is
    // up.
    // Zero for the wrist potentiometer value is parallel to the arm and with
    // the shooter wheels pointed towards the shoulder joint.  This is measured
    // relative to the arm, not the ground, not like the world we actually
    // present to users.
    .frc971.PotAndIndexPosition intake;
    .frc971.PotAndIndexPosition shoulder;
    .frc971.PotAndIndexPosition wrist;
  };

  message Output {
    float voltage_intake;
    float voltage_shoulder;
    float voltage_wrist;

    float voltage_top_rollers;
    float voltage_bottom_rollers;

    // Voltage to sent to the climber. Positive is pulling the robot up.
    float voltage_climber;
    // If true, release the latch to trigger the climber to unfold.
    bool unfold_climber;

    // If true, release the latch to hold the traverse mechanism in the middle.
    bool traverse_unlatched;
    // If true, fire the traverse mechanism down.
    bool traverse_down;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group SuperstructureQueue superstructure_queue;
