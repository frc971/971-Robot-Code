package y2018.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct IntakeSideStatus {
  // Is the subsystem zeroed?
  bool zeroed;

  // The state of the subsystem, if applicable.
  int32_t state;

  // If true, we have aborted.
  bool estopped;

  // Estimated position of the spring.
  float spring_position;
  // Estimated velocity of the spring in units/second.
  float spring_velocity;

  // Estimated position of the joint.
  float motor_position;
  // Estimated velocity of the joint in units/second.
  float motor_velocity;

  // Goal position of the joint.
  float goal_position;
  // Goal velocity of the joint in units/second.
  float goal_velocity;

  // The calculated velocity with delta x/delta t
  float calculated_velocity;

  // The voltage given last cycle;
  float delayed_voltage;

  // State of the estimator.
  .frc971.AbsoluteEstimatorState estimator_state;
};

struct IntakeGoal {
  double roller_voltage;

  // Goal angle in radians of the intake.
  // Zero radians is where the intake is pointing straight out, with positive
  // radians inward towards the cube.
  double left_intake_angle;
  double right_intake_angle;
};

struct IntakeElasticSensors {
  // Position of the motor end of the series elastic in radians.
  .frc971.PotAndAbsolutePosition motor_position;

  // Displacement of the spring in radians.
  double spring_angle;

  // False if the beam break sensor isn't triggered, true if the beam breaker is
  // triggered.
  bool beam_break;
};

struct ArmStatus {
  // State of the estimators.
  .frc971.AbsoluteEstimatorState proximal_estimator_state;
  .frc971.AbsoluteEstimatorState distal_estimator_state;

  // The node we are currently going to.
  uint32_t current_node;
  // Distance (in radians) to the end of the path.
  float path_distance_to_go;
  // Goal position and velocity (radians)
  float goal_theta0;
  float goal_theta1;
  float goal_omega0;
  float goal_omega1;

  // Current position and velocity (radians)
  float theta0;
  float theta1;

  float omega0;
  float omega1;

  // Estimated voltage error for the two joints.
  float voltage_error0;
  float voltage_error1;

  // True if we are zeroed.
  bool zeroed;

  // True if the arm is zeroed.
  bool estopped;

  // The current state machine state.
  uint32_t state;

  uint32_t grab_state;

  // The number of times the LQR solver failed.
  uint32_t failed_solutions;
};

struct ArmPosition {
  // Values of the encoder and potentiometer at the base of the proximal
  // (connected to drivebase) arm in radians.
  .frc971.PotAndAbsolutePosition proximal;

  // Values of the encoder and potentiometer at the base of the distal
  // (connected to proximal) arm in radians.
  .frc971.PotAndAbsolutePosition distal;
};

struct IntakeVoltage {
  // Voltage of the motors on the series elastic on one side (left or right) of
  // the intake.
  double voltage_elastic;

  // Voltage of the rollers on one side (left or right) of the intake.
  double voltage_rollers;
};

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    IntakeGoal intake;

    // Used to identiy a position in the planned set of positions on the arm.
    uint32_t arm_goal_position;
    // If true, start the grab box sequence.
    bool grab_box;

    bool open_claw;

    bool deploy_fork;
  };

  message Status {
    // Are all the subsystems zeroed?
    bool zeroed;

    // If true, any of the subsystems have aborted.
    bool estopped;

    // Status of both intake sides.
    IntakeSideStatus left_intake;
    IntakeSideStatus right_intake;

    ArmStatus arm;
  };

  message Position {
    // Values of the series elastic encoders on the left side of the robot from
    // the rear perspective in radians.
    IntakeElasticSensors left_intake;

    // Values of the series elastic encoders on the right side of the robot from
    // the rear perspective in radians.
    IntakeElasticSensors right_intake;

    ArmPosition arm;

    // Value of the beam breaker sensor. This value is true if the beam is
    // broken, false if the beam isn't broken.
    bool claw_beambreak_triggered;
    // Value of the beambreak sensor detecting when the box has hit the frame
    // cutout.
    bool box_back_beambreak_triggered;
  };

  message Output {
    // Voltage sent to the parts on the left side of the intake.
    IntakeVoltage left_intake;

    // Voltage sent to the parts on the right side of the intake.
    IntakeVoltage right_intake;

    // Voltage sent to the motors on the proximal joint of the arm.
    double voltage_proximal;

    // Voltage sent to the motors on the distal joint of the arm.
    double voltage_distal;

    // Clamped (when true) or unclamped (when false) status sent to the
    // pneumatic claw on the arm.
    bool claw_grabbed;

    // If true, release the arm brakes.
    bool release_arm_brake;
    // If true, release the hook
    bool hook_release;
    // If true, release the forks
    bool forks_release;
  };

  queue Goal goal;
  queue Output output;
  queue Status status;
  queue Position position;
};

queue_group SuperstructureQueue superstructure_queue;
