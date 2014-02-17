package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

struct HalfClawPosition {
  // The current position of this half of the claw.
  double position;
  // The value of the front hall effect sensor.
  bool front_hall_effect;
  // The number of positive and negative edges that have been captured on the
  // front hall effect sensor.
  int32_t front_hall_effect_posedge_count;
  int32_t front_hall_effect_negedge_count;
  // The value of the calibration hall effect sensor.
  bool calibration_hall_effect;
  // The number of positive and negative edges that have been captured on the
  // calibration hall effect sensor.
  int32_t calibration_hall_effect_posedge_count;
  int32_t calibration_hall_effect_negedge_count;
  // The value of the back hall effect sensor.
  bool back_hall_effect;
  // The number of positive and negative edges that have been captured on the
  // back hall effect sensor.
  int32_t back_hall_effect_posedge_count;
  int32_t back_hall_effect_negedge_count;

  // The encoder value at the last posedge of any of the claw hall effect
  // sensors (front, calibration, or back).
  double posedge_value;
  // The encoder value at the last negedge of any of the claw hall effect
  // sensors (front, calibration, or back).
  double negedge_value;
};

// All angles here are 0 horizontal, positive up.
queue_group ClawGroup {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // The angle of the bottom claw.
    double bottom_angle;
    // How much higher the top claw is.
    double seperation_angle;
    bool intake;
  };

  message Position {
    // All the top claw information.
    HalfClawPosition top;
    // All the bottom claw information.
    HalfClawPosition bottom;
  };

  message Output {
    double intake_voltage;
    double top_claw_voltage;
    double bottom_claw_voltage;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue aos.control_loops.Status status;
};

queue_group ClawGroup claw_queue_group;
