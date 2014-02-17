package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group ShooterGroup {
  implements aos.control_loops.ControlLoop;

  message Output {
    double voltage;
    // true: latch engaged, false: latch open
    bool latch_piston;
	// true: brake engaged false: brake released
    bool brake_piston;
  };
  message Goal {
    // encoder ticks of shot energy.
    double shot_power;
    // Shoots as soon as this is true.
    bool shot_requested;
    bool unload_requested;
  };
  message Position {
    //  Gets triggered when the plunger is latched.
    HallEffectStruct plunger;
    // Gets triggered when the pusher is all the way back.
	HallEffectStruct pusher_distal;
    // Triggers just before pusher_distal.
	HallEffectStruct pusher_proximal;
    // Triggers when the latch engages.
	HallEffectStruct latch;

    // In meters, out is positive.
    double position;

	// position at the last positive edge of either of the pusher hall effects.
	double pusher_posedge_value;
	// position at the last negative edge of either of the pusher hall effects.
	double pusher_negedge_value;
  };
  message Status {
    // Whether it's ready to shoot right now.
    bool ready;
    // Whether the plunger is in and out of the way of grabbing a ball.
    bool cocked;
    // How many times we've shot.
    int32_t shots;
    bool done;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ShooterGroup shooter_queue_group;
