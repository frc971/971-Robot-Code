package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

// All angles here are 0 horizontal, positive up.
queue_group ClawLoop {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // The angle of the bottom wrist.
    double bottom_angle;
    // How much higher the top wrist is.
    double seperation_angle;
    bool intake;
  };
  message Position {
    double top_position;

    bool top_front_hall_effect;
    int32_t top_front_hall_effect_posedge_count;
    int32_t top_front_hall_effect_negedge_count;
    bool top_calibration_hall_effect;
    int32_t top_calibration_hall_effect_posedge_count;
    int32_t top_calibration_hall_effect_negedge_count;
    bool top_back_hall_effect;
    int32_t top_back_hall_effect_posedge_count;
    int32_t top_back_hall_effect_negedge_count;
    double top_posedge_value;
    double top_negedge_value;

    double bottom_position;
    bool bottom_front_hall_effect;
    int32_t bottom_front_hall_effect_posedge_count;
    int32_t bottom_front_hall_effect_negedge_count;
    bool bottom_calibration_hall_effect;
    int32_t bottom_calibration_hall_effect_posedge_count;
    int32_t bottom_calibration_hall_effect_negedge_count;
    bool bottom_back_hall_effect;
    int32_t bottom_back_hall_effect_posedge_count;
    int32_t bottom_back_hall_effect_negedge_count;
    double bottom_posedge_value;
    double bottom_negedge_value;
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

queue_group ClawLoop claw;
