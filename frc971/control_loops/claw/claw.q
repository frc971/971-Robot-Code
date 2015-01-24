package frc971.control_loops;

import "aos/common/controls/control_loops.q";

queue_group Claw {
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

  message Goal {
    // Angle of shoulder joint.
    double angle;
    // Voltage of intake rollers. Positive means sucking in, negative means
    // spitting out.
    double intake;

    // Should claw be in open or closed position? (true means open.)
    bool open;
  };

  message Position {
    // Position of shoulder joint from encoder.
    double encoder_pos;
    // Reading from potentiometer.
    double pot_pos;
    // Position of encoder at last index pulse.
    double last_index;
    // Reading from potentiometer at last index pulse.
    double last_index_pot;
    // A count of how many index pulses we've seen on the shoulder encoder.
    uint32_t index_pulses;
  };

  message Output {
    // Voltage for intake motors. Positive is sucking in, negative is spitting
    // out.
    double intake_voltage;
    // Voltage for shoulder motors.
    double shoulder_voltage;
    // Claw in opened or closed position. (true means open.)
    bool open;
  };

  message Status {
    // Is claw zeroed?
    bool zeroed;
    // Has claw zeroed and reached goal?
    bool done;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group Claw claw;
