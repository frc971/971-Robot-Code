package y2014.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group ShooterQueue {
  implements aos.control_loops.ControlLoop;

  message Output {
    double voltage;
    // true: latch engaged, false: latch open
    bool latch_piston;
    // true: brake engaged false: brake released
    bool brake_piston;
  };
  message Goal {
    // Shot power in joules.
    double shot_power;
    // Shoots as soon as this is true.
    bool shot_requested;
    bool unload_requested;
    bool load_requested;
  };

  // Back is when the springs are all the way stretched.
  message Position {
    // In meters, out is positive.
    double position;

    // If the latch piston is fired and this hall effect has been triggered, the
    // plunger is all the way back and latched.
    bool plunger;
    // Gets triggered when the pusher is all the way back.
    .frc971.PosedgeOnlyCountedHallEffectStruct pusher_distal;
    // Triggers just before pusher_distal.
    .frc971.PosedgeOnlyCountedHallEffectStruct pusher_proximal;
    // Triggers when the latch engages.
    bool latch;
  };
  message Status {
    // Whether it's ready to shoot right now.
    bool ready;
    // Whether the plunger is in and out of the way of grabbing a ball.
    // TODO(ben): Populate these!
    bool cocked;
    // How many times we've shot.
    int32_t shots;
    bool done;
    // What we think the current position of the hard stop on the shooter is, in
    // shot power (Joules).
    double hard_stop_power;

    double absolute_position;
    double absolute_velocity;
    uint32_t state;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ShooterQueue shooter_queue;

struct ShooterStateToLog {
	double absolute_position;
	double absolute_velocity;
	uint32_t state;
	bool latch_sensor;
	bool proximal;
	bool distal;
	bool plunger;
	bool brake;
	bool latch_piston;
};

struct ShooterVoltageToLog {
	double X_hat;
	double applied;
};

struct ShooterMovingGoal {
	double dx;
};

struct ShooterChangeCalibration {
	double encoder;
	double real_position;
	double old_position;
	double new_position;
	double old_offset;
	double new_offset;
};

struct ShooterStatusToLog {
	double goal;
	double position;
};

struct PowerAdjustment {
	double requested_power;
	double actual_power;
};
