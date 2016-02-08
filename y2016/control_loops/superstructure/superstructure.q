package y2016.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct JointState {
  // Angle of the joint in radians.
  double angle;
  // Angular velocity of the joint in radians/second.
  float angular_velocity;
  // Profiled goal angle of the joint in radians.
  double goal_angle;
  // Profiled goal angular velocity of the joint in radians/second.
  double goal_angular_velocity;

  // State of the estimator.
  .frc971.EstimatorState estimator_state;
};

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Zero on the intake is when the horizontal tube stock members are level
    // with the ground.  This will be essentially when we are in the intaking
    // position.  Positive is up.  The angle is measured relative to the top
    // of the robot frame.
    // Zero on the shoulder is horizontal.  Positive is up.  The angle is
    // measured relative to the top of the robot frame.
    // Zero on the wrist is horizontal and landed in the bellypan.  Positive is
    // up.  The angle is measured relative to the top of the robot frame.

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
    double voltage_intake;
    double voltage_shoulder;
    double voltage_wrist;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group SuperstructureQueue superstructure_queue;
