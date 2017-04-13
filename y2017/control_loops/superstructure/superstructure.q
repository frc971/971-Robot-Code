package y2017.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/profiled_subsystem.q";
// TODO(austin): Add this back in when the queue compiler supports diamond
// inheritance.
//import "frc971/control_loops/control_loops.q";

struct IntakeGoal {
  // Zero for the intake is when the front tube is tangent with the front of the
  // frame. Positive is out.

  // Goal distance of the intake.
  double distance;

  // Caps on velocity/acceleration for profiling. 0 for the default.
  .frc971.ProfileParameters profile_params;

  // Voltage to send to the rollers. Positive is sucking in.
  double voltage_rollers;

  // If true, disable the intake so we can hang.
  bool disable_intake;

  // The gear servo value.
  double gear_servo;
};

struct IndexerGoal {
  // Indexer angular velocity goals in radians/second.
  double angular_velocity;

  // Roller voltage. Positive is sucking in.
  double voltage_rollers;
};

struct TurretGoal {
  // An angle of zero means the turrent faces toward the front of the
  // robot where the intake is located. The angle increases when the turret
  // turns clockwise (towards right from the front), and decreases when
  // the turrent turns counter-clockwise (towards left from the front).
  // These are from a top view above the robot.
  double angle;

  // If true, ignore the angle and track using vision.  If we don't see
  // anything, we'll use the turret goal above.
  bool track;

  // Caps on velocity/acceleration for profiling. 0 for the default.
  .frc971.ProfileParameters profile_params;
};

struct HoodGoal {
  // Angle the hood is currently at. An angle of zero is at the lower hard
  // stop, angle increases as hood rises.
  double angle;

  // Caps on velocity/acceleration for profiling. 0 for the default.
  .frc971.ProfileParameters profile_params;
};

struct ShooterGoal {
  // Angular velocity goals in radians/second. Positive is shooting out of the
  // robot.
  double angular_velocity;
};

struct IndexerStatus {
  // The current average velocity in radians/second. Positive is moving balls up
  // towards the shooter. This is the angular velocity of the inner piece.
  double avg_angular_velocity;

  // The current instantaneous filtered velocity in radians/second.
  double angular_velocity;

  // True if the indexer is ready.  It is better to compare the velocities
  // directly so there isn't confusion on if the goal is up to date.
  bool ready;

  // True if the indexer is stuck.
  bool stuck;
  float stuck_voltage;

  // The state of the indexer state machine.
  int32_t state;

  // The estimated voltage error from the kalman filter in volts.
  double voltage_error;
  // The estimated voltage error from the stuck indexer kalman filter.
  double stuck_voltage_error;

  // The current velocity measured as delta x / delta t in radians/sec.
  double instantaneous_velocity;

  // The error between our measurement and expected measurement in radians.
  double position_error;
};

struct ShooterStatus {
  // The current average velocity in radians/second.
  double avg_angular_velocity;

  // The current instantaneous filtered velocity in radians/second.
  double angular_velocity;

  // True if the shooter is ready.  It is better to compare the velocities
  // directly so there isn't confusion on if the goal is up to date.
  bool ready;

  // The estimated voltage error from the kalman filter in volts.
  double voltage_error;

  // The current velocity measured as delta x / delta t in radians/sec.
  double instantaneous_velocity;
  double filtered_velocity;
  double fixed_instantaneous_velocity;

  // The error between our measurement and expected measurement in radians.
  double position_error;
};

struct ColumnPosition {
  // Indexer angle in radians relative to the base.  Positive is according to
  // the right hand rule around +z.
  .frc971.HallEffectAndPosition indexer;
  // Turret angle in radians relative to the indexer.  Positive is the same as
  // the indexer.
  .frc971.HallEffectAndPosition turret;
};

struct ColumnEstimatorState {
  bool error;
  bool zeroed;
  .frc971.HallEffectAndPositionEstimatorState indexer;
  .frc971.HallEffectAndPositionEstimatorState turret;
};

struct TurretProfiledSubsystemStatus {
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

  // The estimated voltage error.
  float voltage_error;

  // The calculated velocity with delta x/delta t
  float calculated_velocity;

  // Components of the control loop output
  float position_power;
  float velocity_power;
  float feedforwards_power;

  // State of the estimator.
  ColumnEstimatorState estimator_state;

  double raw_vision_angle;
  double vision_angle;
  bool vision_tracking;

  double turret_encoder_angle;
};

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    IntakeGoal intake;
    IndexerGoal indexer;
    TurretGoal turret;
    HoodGoal hood;
    ShooterGoal shooter;
    bool lights_on;
    bool use_vision_for_shots;
  };

  message Status {
    // Are all the subsystems zeroed?
    bool zeroed;

    // If true, we have aborted. This is the or of all subsystem estops.
    bool estopped;

    // Each subsystems status.
    .frc971.control_loops.AbsoluteProfiledJointStatus intake;
    .frc971.control_loops.IndexProfiledJointStatus hood;
    ShooterStatus shooter;

    TurretProfiledSubsystemStatus turret;
    IndexerStatus indexer;

    float vision_distance;
  };

  message Position {
    // Position of the intake, zero when the intake is in, positive when it is
    // out.
    .frc971.PotAndAbsolutePosition intake;

    // The position of the column.
    ColumnPosition column;

    // The sensor readings for the hood. The units and sign are defined the
    // same as what's in the Goal message.
    .frc971.IndexPosition hood;

    // Shooter wheel angle in radians.
    double theta_shooter;
  };

  message Output {
    // Voltages for some of the subsystems.
    double voltage_intake;
    double voltage_indexer;
    double voltage_shooter;

    // Rollers on the intake.
    double voltage_intake_rollers;
    // Roller on the indexer
    double voltage_indexer_rollers;

    double voltage_turret;
    double voltage_hood;

    double gear_servo;

    // If true, the lights are on.
    bool lights_on;

    bool red_light_on;
    bool green_light_on;
    bool blue_light_on;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group SuperstructureQueue superstructure_queue;
