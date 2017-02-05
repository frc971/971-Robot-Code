package y2017.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct JointState {
  // Position of the joint.
  float position;
  // Velocity of the joint in units/second.
  float velocity;
  // Profiled goal position of the joint.
  float goal_position;
  // Profiled goal velocity of the joint in units/second.
  float goal_velocity;
  // Unprofiled goal position from absoulte zero  of the joint.
  float unprofiled_goal_position;
  // Unprofiled goal velocity of the joint in units/second.
  float unprofiled_goal_velocity;

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

struct IntakeGoal {
  // Zero on the intake is when the intake is retracted inside the robot,
  // unable to intake. Positive is out.

  // Goal distance of the intake.
  double distance;

  // Caps on velocity/acceleration for profiling. 0 for the default.
  .frc971.ProfileParameters profile_params;

  // Voltage to send to the rollers. Positive is sucking in.
  float voltage_rollers;
};

struct SerializerGoal {
  // Serializer angular velocity goals in radians/second.
  double angular_velocity;
};

struct TurretGoal {
  // An angle of zero means the turrent faces toward the front of the
  // robot where the intake is located. The angle increases when the turret
  // turns clockwise (towards right from the front), and decreases when
  // the turrent turns counter-clockwise (towards left from the front).
  // These are from a top view above the robot.
  double angle;

  // Caps on velocity/acceleration for profiling. 0 for the default.
  .frc971.ProfileParameters profile_params;
};

struct HoodGoal {
  // Angle the hood is currently at
  double angle_hood;

  // Caps on velocity/acceleration for profiling. 0 for the default.
  .frc971.ProfileParameters profile_params;
};

struct ShooterGoal {
  // Angular velocity goals in radians/second.
  double angular_velocity;
};

struct IntakeStatus {
  // Is it zeroed?
  bool zeroed;

  // Estimated position and velocities.
  JointState joint_state;

  // If true, we have aborted.
  bool estopped;
};

struct SerializerStatus {
  // The current average velocity in radians/second.
  double avg_angular_velocity;

  // The current instantaneous filtered velocity in radians/second.
  double angular_velocity;

  // True if the serializer is ready.  It is better to compare the velocities
  // directly so there isn't confusion on if the goal is up to date.
  bool ready;

  // If true, we have aborted.
  bool estopped;
};

struct TurretStatus {
  // Is the turret zeroed?
  bool zeroed;

  // If true, we have aborted.
  bool estopped;

  // Estimate angles and angular velocities.
  JointState turret;
};

struct HoodStatus {
  // Is the turret zeroed?
  bool zeroed;

  // If true, we have aborted.
  bool estopped;

  // Estimate angles and angular velocities.
  JointState hood;
};

struct ShooterStatus {
  // The current average velocity in radians/second.
  double avg_angular_velocity;

  // The current instantaneous filtered velocity in radians/second.
  double angular_velocity;

  // True if the shooter is ready.  It is better to compare the velocities
  // directly so there isn't confusion on if the goal is up to date.
  bool ready;

  // If true, we have aborted.
  bool estopped;
};

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    IntakeGoal intake;
    SerializerGoal serializer;
    TurretGoal turret;
    HoodGoal hood;
    ShooterGoal shooter;
  };

  message Status {
    // Are all the subsystems zeroed?
    bool zeroed;

    // If true, we have aborted. This is the or of all subsystem estops.
    bool estopped;

    // Each subsystems status.
    IntakeStatus intake;
    SerializerStatus serializer;
    TurretStatus turret;
    HoodStatus hood;
    ShooterStatus shooter;
  };

  message Position {
    // Position of the intake, zero when the intake is in, positive when it is
    // out.
    .frc971.PotAndAbsolutePosition intake;

    // Serializer angle in radians.
    double theta_serializer;

    // The sensor readings for the turret. The units and sign are defined the
    // same as what's in the Goal message.
    .frc971.PotAndAbsolutePosition turret;

    // Position of the hood in radians
    double theta_hood;

    // Shooter wheel angle in radians.
    double theta_shooter;
  };

  message Output {
    // Voltages for some of the subsystems.
    float voltage_intake;
    float voltage_serializer;
    float voltage_shooter;

    // Rollers on the intake.
    float voltage_intake_rollers;
    // Roller on the serializer
    float voltage_serializer_rollers;

    float voltage_turret;
    float voltage_hood;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group SuperstructureQueue superstructure_queue;
