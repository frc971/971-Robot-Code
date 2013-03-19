package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

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
  };

  message Output {
    float left_voltage;
    float right_voltage;
  };

  message Status {
    bool is_done;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group Drivetrain drivetrain;
