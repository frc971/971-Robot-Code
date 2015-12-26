package y2014_bot3.control_loops;

import "aos/common/controls/control_loops.q";

// For logging information about the state of the shifters.
struct CIMLogging {
  // Whether the code thinks the left side is currently in gear.
  bool left_in_gear;
  // Whether the code thinks the right side is currently in gear.
  bool right_in_gear;
  // The velocity in rad/s (positive forward) the code thinks the left motor
  // is currently spinning at.
  double left_motor_speed;
  // The velocity in rad/s (positive forward) the code thinks the right motor
  // is currently spinning at.
  double right_motor_speed;
  // The velocity estimate for the left side of the robot in m/s (positive
  // forward) used for shifting.
  double left_velocity;
  // The velocity estimate for the right side of the robot in m/s (positive
  // forward) used for shifting.
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
    // Position goal for the left side in meters when the closed-loop controller
    // is active.
    double left_goal;
    // Velocity goal for the left side in m/s when the closed-loop controller
    // is active.
    double left_velocity_goal;
    // Position goal for the right side in meters when the closed-loop
    // controller is active.
    double right_goal;
    // Velocity goal for the right side in m/s when the closed-loop controller
    // is active.
    double right_velocity_goal;
  };

  message Position {
    // Relative position of the left side in meters.
    double left_encoder;
    // Relative position of the right side in meters.
    double right_encoder;
    // The speed in m/s of the left side from the most recent encoder pulse,
    // or 0 if there was no edge within the last 5ms.
    double left_speed;
    // The speed in m/s of the right side from the most recent encoder pulse,
    // or 0 if there was no edge within the last 5ms.
    double right_speed;
  };

  message Output {
    // Voltage to send to the left motor(s).
    double left_voltage;
    // Voltage to send to the right motor(s).
    double right_voltage;
    // True to set the left shifter piston for high gear.
    bool left_high;
    // True to set the right shifter piston for high gear.
    bool right_high;
  };

  message Status {
    // Estimated speed of the center of the robot in m/s (positive forwards).
    double robot_speed;
    // Estimated relative position of the left side in meters.
    double filtered_left_position;
    // Estimated relative position of the right side in meters.
    double filtered_right_position;
    // Estimated velocity of the left side in m/s.
    double filtered_left_velocity;
    // Estimated velocity of the right side in m/s.
    double filtered_right_velocity;

    // The voltage we wanted to send to the left side last cycle.
    double uncapped_left_voltage;
    // The voltage we wanted to send to the right side last cycle.
    double uncapped_right_voltage;
    // True if the output voltage was capped last cycle.
    bool output_was_capped;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group DrivetrainQueue drivetrain_queue;
