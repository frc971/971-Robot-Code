package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

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

queue_group Drivetrain {
  implements aos.control_loops.ControlLoop;

  message Goal {
    float steering;
    float throttle;
    bool highgear;
    bool quickturn;
    bool control_loop_driving;
    float left_goal;
    float left_velocity_goal;
    float right_goal;
    float right_velocity_goal;
  };

  message Position {
    double left_encoder;
    double right_encoder;
    double left_shifter_position;
    double right_shifter_position;
    double battery_voltage;
  };

  message Output {
    float left_voltage;
    float right_voltage;
    bool left_high;
    bool right_high;
  };

  message Status {
    bool is_done;
    double robot_speed;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group Drivetrain drivetrain;
