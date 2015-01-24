package frc971.control_loops;

import "aos/common/controls/control_loops.q";

queue_group Fridge {
  implements aos.control_loops.ControlLoop;

  // NOTE: Unless otherwise specified, assume that all angle measures are in
  // radians. An angle of zero means that the appendage is sticking straight out
  // horizontally, pointing towards the front of the robot. Rotating the appendage
  // up and towards the back of the robot increases the angle, moving it down
  // and towards the back decreases it. (Think unit circle.) This rule holds
  // true for both angle goals and encoder positions.
  // Also note that unless otherwise specified, potentiometer readings are
  // from 0V to 5V. As with the encoders, moving up and towards the back
  // of the robot increases this value, moving down and towards the back
  // decreases it.
  // For all output voltage parameters, assume that a positive voltage moves
  // the appendage in a direction that increases the value of the encoder, and
  // vice versa. (For an output voltage parameter for something without an
  // encoder, directions will be individually specified.)

  // NOTE: Elevator heights are defined as follows: The height of the elevator
  // is the vertical distance (in meters) between the top of the frame
  // (front and back), and the arm pivot axis. A constant specifies the minimum
  // value for this distance.

  message Goal {
    // Position of the arm in radians.
    double angle;
    // Height of the elevator in meters.
    double height;

    // Should the grabbers be deployed?
    bool grabbers_deployed;
  };

  message Position {
    // Position of arm from encoder.
    double arm_encoder_pos;
    // Reading from arm potentiometer.
    double arm_pot_pos;
    // Position of arm encoder at last index pulse.
    double arm_last_index;
    // Reading from arm potentiometer at last index pulse.
    double arm_last_index_pot;
    // A count of how many index pulses we've seen on the arm encoder.
    uint32_t arm_index_pulses;

    // Height of left side from encoder.
    double left_encoder_pos;
    // Reading from left side potentiometer. Directions work the same for this
    // as for the encoder.
    double left_pot_pos;
    // Position of left encoder at last index pulse.
    double left_last_index;
    // Reading from left potentiometer at last index pulse.
    double left_last_index_pot;
    // A count of how many index pulses we've seen on the left encoder.
    uint32_t left_index_pulses;

    // Height of right side from encoder. Directions are the same as
    // for the left side.
    double right_encoder_pos;
    // Reading from right side potentiometer. Directions work the same for this
    // as for the encoder.
    double right_pot_pos;
    // Position of right encoder at last index pulse.
    double right_last_index;
    // Reading from right potentiometer at last index pulse.
    double right_last_index_pot;
    // A count of how many index pulses we've seen on the right encoder.
    uint32_t right_index_pulses;

  };

  message Status {
    // Are both the arm and elevator zeroed?
    bool zeroed;
    // Are we zeroed and have reached our goal position on both the arm and
    // elevator?
    bool done;
  };

  message Output {
    // Voltage of arm motor.
    double arm_voltage;
    // Voltage of left elevator motor.
    double left_voltage;
    // Voltage of right elevator motor.
    double right_voltage;

    // Are grabber pistons deployed?
    bool grabbers_deployed;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};
