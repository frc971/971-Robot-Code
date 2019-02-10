package y2019.control_loops.superstructure;

import "aos/controls/control_loops.q";
import "frc971/control_loops/profiled_subsystem.q";

struct ElevatorGoal {
  // Meters, 0 = lowest position - mechanical hard stop,
  // positive = upward
  double height;

  .frc971.ProfileParameters profile_params;
};

struct IntakeGoal {
  // Positive is rollers intaking inward.
  double roller_voltage;

  // 0 = linkage on the sprocket is pointing straight up,
  // positive = forward
  double joint_angle;

  .frc971.ProfileParameters profile_params;
};

struct SuctionGoal {
  // True = open solenoid (apply suction)
  // Top/bottom are when wrist is forward
  bool top;
  bool bottom;
};

struct StiltsGoal {
  // Distance stilts extended out of the bottom of the robot. Positive = down.
  // 0 is the height such that the bottom of the stilts is tangent to the bottom
  // of the middle wheels.
  double height;

  .frc971.ProfileParameters profile_params;
};

struct WristGoal {
  // 0 = Straight up parallel to elevator
  // Positive rotates toward intake from 0
  double angle;
  .frc971.ProfileParameters profile_params;
};

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    ElevatorGoal elevator;
    IntakeGoal intake;
    SuctionGoal suction;
    StiltsGoal stilts;
    WristGoal wrist;
  };

  message Status {
    // All subsystems know their location.
    bool zeroed;

    // If true, we have aborted. This is the or of all subsystem estops.
    bool estopped;

    // Whether suction_pressure indicates cargo is held
    bool has_piece;

    // Status of each subsystem.
    .frc971.control_loops.PotAndAbsoluteEncoderProfiledJointStatus elevator;
    .frc971.control_loops.PotAndAbsoluteEncoderProfiledJointStatus wrist;
    .frc971.control_loops.PotAndAbsoluteEncoderProfiledJointStatus intake;
    .frc971.control_loops.PotAndAbsoluteEncoderProfiledJointStatus stilts;
  };

  message Position {
    // Input from pressure sensor in psi
    // 0 = current atmospheric pressure, negative = suction.
    double suction_pressure;

    // Position of the elevator, 0 at lowest position, positive when up.
    .frc971.PotAndAbsolutePosition elevator;

    // Position of wrist, 0 when up, positive is rotating toward the front,
    // over the top.
    .frc971.PotAndAbsolutePosition wrist;

    // Position of the intake. 0 when rollers are retracted, positive extended.
    .frc971.AbsolutePosition intake_joint;

    // Position of the stilts, 0 when retracted (defualt), positive lifts robot.
    .frc971.PotAndAbsolutePosition stilts;
  };

  message Output {
    // Voltage sent to motors moving elevator up/down. Positive is up.
    double elevator_voltage;

    // Voltage sent to wrist motors on elevator to rotate.
    // Positive rotates over the top towards the front of the robot.
    double wrist_voltage;

    // Voltage sent to motors on intake joint. Positive extends rollers.
    double intake_joint_voltage;

    // Voltage sent to rollers on intake. Positive rolls inward.
    double intake_roller_voltage;

    // Voltage sent to motors to move stilts height. Positive moves robot upward.
    double stilts_voltage;

    // True opens solenoid (applies suction)
    // Top/bottom are when wrist is toward the front of the robot
    bool intake_suction_top;
    bool intake_suction_bottom;

    // Voltage sent to the vacuum pump motors.
    double pump_voltage;
  };

  queue Goal goal;
  queue Output output;
  queue Status status;
  queue Position position;
};

queue_group SuperstructureQueue superstructure_queue;
