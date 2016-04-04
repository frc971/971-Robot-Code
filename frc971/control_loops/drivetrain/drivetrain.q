package frc971.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

// For logging information about what the code is doing with the shifters.
struct GearLogging {
  // Which controller is being used.
  int8_t controller_index;

  // Whether each loop for the drivetrain sides is the high-gear one.
  bool left_loop_high;
  bool right_loop_high;

  // The states of each drivetrain shifter.
  int8_t left_state;
  int8_t right_state;
};

// For logging information about the state of the shifters.
struct CIMLogging {
  // Whether the code thinks each drivetrain side is currently in gear.
  bool left_in_gear;
  bool right_in_gear;

  // The angular velocities (in rad/s, positive forward) the code thinks motors
  // on each side of the drivetrain are moving at.
  double left_motor_speed;
  double right_motor_speed;

  // The velocity estimates for each drivetrain side of the robot (in m/s,
  // positive forward) that can be used for shifting.
  double left_velocity;
  double right_velocity;
};

queue_group DrivetrainQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Position of the steering wheel (positive = turning left when going
    // forwards).
    double steering;

    // Position of the throttle (positive forwards).
    double throttle;

    // True to shift into high, false to shift into low.
    bool highgear;

    // True to activate quickturn.
    bool quickturn;

    // True to have the closed-loop controller take over.
    bool control_loop_driving;

    // Position goals for each drivetrain side (in meters) when the
    // closed-loop controller is active.
    double left_goal;
    double right_goal;

    // Velocity goal for each drivetrain side (in m/s) when the closed-loop
    // controller is active.
    double left_velocity_goal;
    double right_velocity_goal;

    // Motion profile parameters.
    // The control loop will profile if these are all non-zero.
    .frc971.ProfileParameters linear;
    .frc971.ProfileParameters angular;
  };

  message Position {
    // Relative position of each drivetrain side (in meters).
    double left_encoder;
    double right_encoder;

    // The speed in m/s of each drivetrain side from the most recent encoder
    // pulse, or 0 if there was no edge within the last 5ms.
    double left_speed;
    double right_speed;

    // Position of each drivetrain shifter, scaled from 0.0 to 1.0 where smaller
    // is towards low gear.
    double left_shifter_position;
    double right_shifter_position;

    // Raw analog voltages of each shifter hall effect for logging purposes.
    double low_left_hall;
    double high_left_hall;
    double low_right_hall;
    double high_right_hall;
  };

  message Output {
    // Voltage to send to motor(s) on either side of the drivetrain.
    double left_voltage;
    double right_voltage;

    // Whether to set each shifter piston to high gear.
    bool left_high;
    bool right_high;
  };

  message Status {
    // Estimated speed of the center of the robot in m/s (positive forwards).
    double robot_speed;

    // Estimated relative position of each drivetrain side (in meters).
    double estimated_left_position;
    double estimated_right_position;

    // Estimated velocity of each drivetrain side (in m/s).
    double estimated_left_velocity;
    double estimated_right_velocity;

    // The voltage we wanted to send to each drivetrain side last cycle.
    double uncapped_left_voltage;
    double uncapped_right_voltage;

    // The goal velocities for the polydrive controller.
    double left_velocity_goal;
    double right_velocity_goal;

    // The voltage error for the left and right sides.
    double left_voltage_error;
    double right_voltage_error;

    // The profiled goal states.
    double profiled_left_position_goal;
    double profiled_right_position_goal;
    double profiled_left_velocity_goal;
    double profiled_right_velocity_goal;

    // The KF offset
    double estimated_angular_velocity_error;
    // The KF estimated heading.
    double estimated_heading;
    // The KF wheel estimated heading.
    //double estimated_wheel_heading;

    // True if the output voltage was capped last cycle.
    bool output_was_capped;

    // The angle of the robot relative to the ground.
    double ground_angle;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group DrivetrainQueue drivetrain_queue;
