package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

struct Claw {
  double position;
  bool front_hall_effect;
  int32_t front_hall_effect_posedge_count;
  int32_t front_hall_effect_negedge_count;
  bool calibration_hall_effect;
  int32_t calibration_hall_effect_posedge_count;
  int32_t calibration_hall_effect_negedge_count;
  bool back_hall_effect;
  int32_t back_hall_effect_posedge_count;
  int32_t back_hall_effect_negedge_count;

  // The encoder value at the last posedge of any of the claw hall effect
  // sensors.
  double posedge_value;
  // The encoder value at the last negedge of any of the claw hall effect
  // sensors.
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
    // Top claw position relative to power on.
    //double top_position;

    Claw top;
    Claw bottom;

    // Three Hall Effects with respect to the top claw
    //bool top_front_hall_effect;
    //int32_t top_front_hall_effect_posedge_count;
    //int32_t top_front_hall_effect_negedge_count;
    //bool top_calibration_hall_effect;
    //int32_t top_calibration_hall_effect_posedge_count;
    //int32_t top_calibration_hall_effect_negedge_count;
    //bool top_back_hall_effect;
    //int32_t top_back_hall_effect_posedge_count;
    //int32_t top_back_hall_effect_negedge_count;

    // The encoder value at the last posedge of any of the top claw hall effect
    // sensors.
    //double top_posedge_value;
    // The encoder value at the last negedge of any of the top claw hall effect
    // sensors.
    //double top_negedge_value;

    // bottom claw relative position
    //double bottom_position;

    // Three Hall Effects with respect to the bottom claw
    //bool bottom_front_hall_effect;
    //int32_t bottom_front_hall_effect_posedge_count;
    //int32_t bottom_front_hall_effect_negedge_count;
    //bool bottom_calibration_hall_effect;
    //int32_t bottom_calibration_hall_effect_posedge_count;
    //int32_t bottom_calibration_hall_effect_negedge_count;
    //bool bottom_back_hall_effect;
    //int32_t bottom_back_hall_effect_posedge_count;
    //int32_t bottom_back_hall_effect_negedge_count;

    // The encoder value at the last posedge of any of the bottom claw hall
    // effect sensors.
    //double bottom_posedge_value;
    // The encoder value at the last negedge of any of the bottom claw hall
    // effect sensors.
    //double bottom_negedge_value;
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
