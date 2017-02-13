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
};

struct IndexerGoal {
  // Indexer angular velocity goals in radians/second.
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

  // If true, we have aborted.
  bool estopped;
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
    IndexerGoal indexer;
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
    .frc971.control_loops.ProfiledJointStatus intake;
    .frc971.control_loops.ProfiledJointStatus turret;
    .frc971.control_loops.ProfiledJointStatus hood;
    IndexerStatus indexer;
    ShooterStatus shooter;
  };

  message Position {
    // TODO(austin): The turret and intake really should be absolute.  Switch
    // them over when that class is ready.

    // Position of the intake, zero when the intake is in, positive when it is
    // out.
    .frc971.PotAndIndexPosition intake;

    // Indexer angle in radians.
    double theta_indexer;

    // The sensor readings for the turret. The units and sign are defined the
    // same as what's in the Goal message.
    .frc971.PotAndIndexPosition turret;

    // The sensor readings for the hood. The units and sign are defined the
    // same as what's in the Goal message.
    .frc971.PotAndIndexPosition hood;

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
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group SuperstructureQueue superstructure_queue;
