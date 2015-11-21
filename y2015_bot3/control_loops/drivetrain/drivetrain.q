package y2015_bot3.control_loops;

import "aos/common/controls/control_loops.q";

struct GearLogging {
  int8_t controller_index;
  bool left_loop_high;
  bool right_loop_high;
  int8_t left_state;
  int8_t right_state;
};

struct CIMLogging {
  bool left_in_gear;
  bool right_in_gear;
  double left_motor_speed;
  double right_motor_speed;
  double left_velocity;
  double right_velocity;
};

queue_group DrivetrainQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    double steering;
    double throttle;
    //bool highgear;
    bool quickturn;
    bool control_loop_driving;
    double left_goal;
    double left_velocity_goal;
    double right_goal;
    double right_velocity_goal;
  };

  message Position {
    double left_encoder;
    double right_encoder;
    //double left_shifter_position;
    //double right_shifter_position;
  };

  message Output {
    double left_voltage;
    double right_voltage;
    bool left_high;
    bool right_high;
  };

  message Status {
    double robot_speed;
    double filtered_left_position;
    double filtered_right_position;
    double filtered_left_velocity;
    double filtered_right_velocity;

    double uncapped_left_voltage;
    double uncapped_right_voltage;
    bool output_was_capped;

    bool is_done;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group DrivetrainQueue drivetrain_queue;
