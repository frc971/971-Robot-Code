package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

queue_group IndexLoop {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // The state for the indexer to be in.
    // 0 means hold position, in a low power state.
    // 1 means get ready to load discs by shifting the discs down.
    // 2 means ready the discs, spin up the transfer roller, and accept discs.
    // 3 means get ready to shoot, and place a disc grabbed in the loader.
    // 4 means shoot at will.
    int32_t goal_state;
  };

  message Position {
    // Index position
    double index_position;
    bool bottom_disc_detect;
    bool top_disc_detect;
  };

  message Output {
    // Transfer roller output voltage.
    // Positive means into the robot.
    double transfer_voltage;
    // Index roller.  Positive means up towards the shooter.
    double index_voltage;
    // Loader pistons.
    bool disc_clamped;
    bool loader_up;
    bool disc_ejected;
  };

  message Status {
    // Number of discs in the hopper
    int32_t hopper_disc_count;
    // Number of shot since reboot.
    int32_t total_disc_count;
    // Disc ready in the loader.
    bool preloaded;
    // Indexer ready to accept more discs.
    bool ready_to_intake;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group IndexLoop index_loop;
