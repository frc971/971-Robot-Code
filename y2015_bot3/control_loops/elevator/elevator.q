package y2015_bot3.control_loops;

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

    // Whether the passive elevator is supporting the stack/can; true means it
    // is supporting; false means it is not.
    bool passive_support;
    // Whether the can support is restraining the can; true means it
    // is supporting; false means it is not.
    bool can_support;
  };

  message Position {
    double encoder;

    // bottom hall effect sensor for zeroing purposes.
    bool bottom_hall_effect;

    // True if there is a tote inside the elevator.
    bool has_tote;
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

    // True if there is a tote inside the elevator.
    bool has_tote;

    // If true, we have aborted.
    bool estopped;

    // The internal state of the state machine.
    int32_t state;
  };

  message Output {
    // Voltage for the active elevator.
    double elevator;

    // Toggle for the passive elevator that supports the stack in the robot.
    // True means support the stack, false means release the support from the
    // stack.
    bool passive_support;
    // Toggle for the that supports the can in the robot.
    // True means support the can, false means release the can.
    bool can_support;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ElevatorQueue elevator_queue;
