package frc971.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

// Represents states for all of the box-grabbing pistons.
// true is grabbed and false is retracted for all of them.
struct GrabberPistons {
  bool top_front;
  bool top_back;
  bool bottom_front;
  bool bottom_back;
};

queue_group FridgeQueue {
  implements aos.control_loops.ControlLoop;

  // All angles are in radians with 0 sticking straight up.
  // Rotating up and into the robot (towards the back
  // where boxes are placed) is positive. Positive output voltage moves all
  // mechanisms in the direction with positive sensor values.

  // Elevator heights are the vertical distance (in meters) from a defined zero.
  // In this case, zero is where the portion of the carriage that Spencer
  // removed lines up with the bolt.

  message Goal {
    // Angle of the arm.
    double angle;
    // Height of the elevator.
    double height;

    // Angular velocity of the arm.
    double angular_velocity;
    // Linear velocity of the elevator.
    double velocity;

    // TODO(austin): Do I need acceleration here too?

    GrabberPistons grabbers;
  };

  message Position {
    PotAndIndexPair arm;
    PotAndIndexPair elevator;
  };

  message Status {
    // Are both the arm and elevator zeroed?
    bool zeroed;

    // Angle of the arm.
    double angle;
    // Height of the elevator.
    double height;
    // state of the grabber pistons
    GrabberPistons grabbers;

    // Goal angle of the arm.
    double goal_angle;
    // Goal height of the elevator.
    double goal_height;

    // If true, we have aborted.
    bool estopped;

    // The internal state of the state machine.
    int32_t state;

    EstimatorState left_elevator_state;
    EstimatorState right_elevator_state;
    EstimatorState left_arm_state;
    EstimatorState right_arm_state;
  };

  message Output {
    double left_arm;
    double right_arm;
    double left_elevator;
    double right_elevator;

    GrabberPistons grabbers;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group FridgeQueue fridge_queue;
