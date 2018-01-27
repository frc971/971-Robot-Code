package y2018.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct JointStatus {
  // Is the subsystem zeroed?
  bool zeroed;

  // The state of the subsystem, if applicable.  -1 otherwise.
  int32_t state;

  // If true, we have aborted.
  bool estopped;

  // Position of the joint.
  float position;
  // Velocity of the joint in units/second.
  float velocity;
  // Profiled goal position of the joint.
  float goal_position;
  // Profiled goal velocity of the joint in units/second.
  float goal_velocity;
  // Unprofiled goal position from absoulte zero of the joint.
  float unprofiled_goal_position;
  // Unprofiled goal velocity of the joint in units/second.
  float unprofiled_goal_velocity;

  // The estimated disturbance torque.
  float disturbance_torque;

  // The calculated velocity with delta x/delta t
  float calculated_velocity;

  // Components of the control loop output
  float position_power;
  float velocity_power;
  float feedforwards_power;

  // State of the estimator.
  .frc971.AbsoluteEstimatorState estimator_state;
};

struct IntakeGoal {
  float roller_voltage;

  // Goal angle in radians of the intake.
  // Zero radians is where the intake is pointing straight out, with positive
  // radians inward towards the cube.
  double intake_angle;
};

struct ArmGoal {
  // Target X distance (paralell to the ground) of the arm endpoint from the
  // base of the proximal arm (connected to the drivetrain) in meters with
  // positive meters towards the front of the robot.
  double proximal_position;

  // Target Y distance (perpendicular to the ground) of the arm endpoint from
  // the base of the distal arm (connected to the proximal arm) in meters with
  // positive meters towards the front of the robot.
  double distal_position;
};

struct IntakeElasticEncoders {
  // Values of the encoder connected to the motor end of the series elastic in
  // radians.
  .frc971.IndexPosition motor_encoder;

  // Values of the encoder connected to the load end of the series elastic in
  // radians.
  .frc971.IndexPosition spring_encoder;
};

struct IntakePosition {
  // False if the beam breaker isn't triggered, true if the beam breaker is
  // triggered.
  bool left_beam_breaker;

  // False if the beam breaker isn't triggered, true if the beam breaker is
  // triggered.
  bool right_beam_breaker;

  // Values of the series elastic encoders on the left side of the robot from
  // the rear perspective in radians.
  IntakeElasticEncoders left;

  // Values of the series elastic encoders on the right side of the robot from
  // the rear perspective in radians.
  IntakeElasticEncoders right;
};

struct ArmPosition {
  // Values of the encoder and potentiometer at the base of the proximal
  // (connected to drivebase) arm in radians.
  .frc971.PotAndIndexPosition proximal;

  // Values of the encoder and potentiometer at the base of the distal
  // (connected to proximal) arm in radians.
  .frc971.PotAndIndexPosition distal;
};

struct ClawPosition {
  // Value of the beam breaker sensor. This value is true if the beam is broken,
  // false if the beam isn't broken.
  bool beam_triggered;
};

struct IntakeVoltage {
  // Voltage of the motors on the series elastic on one side (left or right) of
  // the intake.
  double voltage_elastic;

  // Voltage of the rollers on one side (left or right) of the intake.
  double voltage_rollers;
};

struct IntakeOutput {
  // Voltage sent to the parts on the left side of the intake.
  IntakeVoltage left;

  // Voltage sent to the parts on the right side of the intake.
  IntakeVoltage right;
};


queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    IntakeGoal intake;
    ArmGoal arm;

    bool open_claw;

    bool deploy_fork;
  };

  message Status {
    // Are all the subsystems zeroed?
    bool zeroed;

    // If true, any of the subsystems have aborted.
    bool estopped;

    // Estimated angles and angular velocities of the superstructure subsystems.
    JointStatus arm;

    JointStatus intake;

    bool claw_open;
  };

  message Position {
    IntakePosition intake;
    ArmPosition arm;
    ClawPosition claw;
  };

  message Output {
    IntakeOutput intake;

    // Placehodler for when we figure out how we are extending and retracting
    // the fork tines.
    bool tines_output;

    // Placeholder for when we figure out how we are deploying and retracting
    // the fork.
    bool fork_output;

    // Voltage sent to the motors on the proximal joint of the arm.
    double voltage_proximal;

    // Voltage sent to the motors on the distal joint of the arm.
    double voltage_distal;

    // Clamped (when true) or unclamped (when false) status sent to the
    // pneumatic claw on the arm.
    bool claw_output;
  };

  queue Goal goal;
  queue Output output;
  queue Status status;
  queue Position position;
};

queue_group SuperstructureQueue superstructure_queue;
