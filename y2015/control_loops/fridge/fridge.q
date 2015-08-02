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

  // X/Y positions are distances (in meters) the fridge is away from its origin
  // position. Origin is considered at a height of zero and an angle of zero.
  // Positive X positions are towards the front of the robot and negative X is
  // towards the back of the robot (which is where we score).
  // Y is positive going up and negative when it goes below the origin.

  message Goal {
    // Profile type.
    // Set to 0 for angle/height profiling.
    // Set to 1 for x/y profiling.
    int32_t profiling_type;

    // Angle of the arm.
    double angle;
    // Height of the elevator.
    double height;

    // Angular velocity of the arm.
    float angular_velocity;
    // Linear velocity of the elevator.
    float velocity;

    // Maximum arm profile angular velocity or 0 for the default.
    float max_angular_velocity;
    // Maximum elevator profile velocity or 0 for the default.
    float max_velocity;

    // Maximum arm profile acceleration or 0 for the default.
    float max_angular_acceleration;
    // Maximum elevator profile acceleration or 0 for the default.
    float max_acceleration;

    // X position of the fridge.
    double x;
    // Y position of the fridge.
    double y;

    // Velocity of the x position of the fridge.
    float x_velocity;
    // Velocity of the y position of the fridge.
    float y_velocity;

    // Maximum x profile velocity or 0 for the default.
    float max_x_velocity;
    // Maximum y profile velocity or 0 for the default.
    float max_y_velocity;

    // Maximum x profile acceleration or 0 for the default.
    float max_x_acceleration;
    // Maximum y profile acceleration or 0 for the default.
    float max_y_acceleration;

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

    // Estimated angle of the arm.
    double angle;
    // Estimated angular velocity of the arm.
    float angular_velocity;
    // Estimated height of the elevator.
    double height;
    // Estimated velocity of the elvator.
    float velocity;
    // state of the grabber pistons
    GrabberPistons grabbers;

    // Goal angle and velocity of the arm.
    double goal_angle;
    float goal_angular_velocity;
    // Goal height and velocity of the elevator.
    double goal_height;
    float goal_velocity;

    // Estimated X/Y position of the fridge.
    // These are translated directly from the height/angle statuses.
    double x;
    double y;
    float x_velocity;
    float y_velocity;

    // X/Y goals of the fridge.
    // These are translated directly from the height/angle goals.
    double goal_x;
    double goal_y;
    float goal_x_velocity;
    float goal_y_velocity;

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
