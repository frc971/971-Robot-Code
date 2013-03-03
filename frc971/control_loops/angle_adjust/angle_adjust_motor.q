package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

queue_group AngleAdjustLoop {
  implements aos.control_loops.ControlLoop;

  message Position {
    // Angle of the encoder.
    double before_angle;
    double after_angle;
    bool bottom_hall_effect;
    bool middle_hall_effect;
    // The exact position when the corresponding hall_effect changed.
    double bottom_calibration;
    double middle_calibration;
  };

  queue aos.control_loops.Goal goal;
  queue Position position;
  queue aos.control_loops.Output output;
  queue aos.control_loops.Status status;
};

queue_group AngleAdjustLoop angle_adjust;
