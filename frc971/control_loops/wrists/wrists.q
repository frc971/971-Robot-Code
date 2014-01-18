package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

// All angles here are 0 horizontal, positive up.
queue_group WristsLoop {
  implements aos.control_loops.ControlLoop;

  message Goal {
	// The angle of the bottom wrist.
	double bottom_angle;
	// How much higher the top wrist is.
	double between_angle;
	bool intake;
  };
  message Position {
    double bottom_position, top_position;
	bool bottom_hall_effect, top_hall_effect;
	double bottom_calibration, top_calibration;
  };

  queue Goal goal;
  queue Position position;
  queue aos.control_loops.Output output;
  queue aos.control_loops.Status status;
};

queue_group WristsLoop wrists;
