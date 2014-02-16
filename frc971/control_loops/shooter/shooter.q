package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

queue_group ShooterLoop {
  implements aos.control_loops.ControlLoop;

  message Output {
    // The energy to load to in joules.
    double voltage;
    // Shoots as soon as this is true.
    bool latched;
    bool locked; //Disc brake locked
  };
  message Goal {
    // encoder ticks of shot energy.
    double shot_power;
    double goal;
    // Shoots as soon as this is true.
    bool shot_requested;
    bool unload_requested;
  };
  message Position {
	// back on the plunger
    bool plunger_back_hall_effect;
	// truely back on the pusher
	bool pusher_distal_hall_effect;
	// warning that we are back on the pusher
	bool pusher_proximal_hall_effect;
	// the latch is closed
	bool latch_hall_effect;

	// count of positive edges
	int64_t plunger_back_hall_effect_posedge_count;
	// count of negative edges
	int64_t plunger_back_hall_effect_negedge_count;
	// count of positive edges
	int64_t pusher_distal_hall_effect_posedge_count;
	// count of negative edges
	int64_t pusher_distal_hall_effect_negedge_count;
	// count of positive edges
	int64_t pusher_proximal_hall_effect_posedge_count;
	// count of negative edges
	int64_t pusher_proximal_hall_effect_negedge_count;
	// count of positive edges
	int64_t latch_hall_effect_posedge_count;
	// count of negative edges
	int64_t latch_hall_effect_negedge_count;

    // In meters, out is positive.
    double position;
    double back_calibration;

	// last positive edge
	double posedge_value;
	// last negative edge
	double negedge_value;
  };
  // I don't think this is needed, but it is here
  // so I won't delete it yet.
  message Status {
    // Whether it's ready to shoot right now.
    bool ready;
    // Whether the plunger is in and out of the way of grabbing a ball.
    bool cocked;
    // How many times we've shot.
    int32_t shots;
    bool done;
  };

  message Output {
	// desired motor voltage
    double voltage;
	// true: close latch, false: open latch
    double latch_piston;
	// true: brake engaded, false: brake release
    double brake_piston;
  };
  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ShooterLoop shooter_queue_group;
