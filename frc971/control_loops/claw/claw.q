package frc971.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group ClawQueue {
  implements aos.control_loops.ControlLoop;

  // All angles are in radians with 0 sticking straight out the front. Rotating
  // up and into the robot is positive. Positive output voltage moves in the
  // direction of positive encoder values.

  message Goal {
    // Angle of wrist joint.
    double angle;
    // Angular velocity of wrist.
    float angular_velocity;
    // Maximum profile velocity, or 0 for the default.
    float max_velocity;
    // Maximum profile acceleration, or 0 for the default.
    float max_acceleration;
    // Voltage of intake rollers. Positive means sucking in, negative means
    // spitting out.
    double intake;

    // true to signal the rollers to close.
    bool rollers_closed;
  };

  message Position {
    PotAndIndexPosition joint;
  };

  message Output {
    // Voltage for intake motors. Positive is sucking in, negative is spitting
    // out.
    double intake_voltage;
    // Voltage for claw motor.
    double voltage;

    // true to signal the rollers to close.
    bool rollers_closed;
  };

  message Status {
    // Is claw zeroed?
    bool zeroed;
    // Did the claw estop?
    bool estopped;
    // The internal state of the claw state machine.
    uint32_t state;
    EstimatorState zeroing_state;

    // Estimated angle of wrist joint.
    double angle;
    // Estimated angular velocity of wrist.
    float angular_velocity;

    // Goal angle of wrist joint.
    double goal_angle;
    float goal_velocity;
    // Voltage of intake rollers. Positive means sucking in, negative means
    // spitting out.
    double intake;

    // True iff there has been enough time since we actuated the rollers outward
    // that they should be there.
    bool rollers_open;
    // True iff there has been enough time since we actuated the rollers closed
    // that they should be there.
    bool rollers_closed;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ClawQueue claw_queue;

