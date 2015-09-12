package bot3.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group ElevatorQueue {
  implements aos.control_loops.ControlLoop;

  // Elevator heights are the vertical distance (in meters) from a defined zero.

  message Goal {
    // Height of the elevator.
    double height;

    // Linear velocity of the elevator.
    float velocity;

    // Maximum elevator profile velocity or 0 for the default.
    float max_velocity;

    // Maximum elevator profile acceleration or 0 for the default.
    float max_acceleration;
  };

  message Position {
    // bottom hall effect sensor.
    bool bottom_hall_effect;
    // encoders used for zeroing.
    double encoder;
  };

  message Status {
    // Is the elevator zeroed?
    bool zeroed;

    // Estimated height of the elevator.
    double height;

    // Estimated velocity of the elevator.
    float velocity;

    // Goal height and velocity of the elevator.
    double goal_height;
    float goal_velocity;

    // If true, we have aborted.
    bool estopped;

    // The internal state of the state machine.
    int32_t state;
  };

  message Output {
    double elevator;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ElevatorQueue elevator_queue;
