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
    PosedgeOnlyCountedHallEffectStruct pusher_distal;
    // Triggers just before pusher_distal.
    PosedgeOnlyCountedHallEffectStruct pusher_proximal;
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
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ShooterGroup shooter_queue_group;
