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
    // 5 means re-initialize
    int32_t goal_state;
    // Forces the loader to fire.
    bool force_fire;
  };

  message Position {
    // Index position
    double index_position;

    // Current values of both sensors.
    bool top_disc_detect;
    bool bottom_disc_detect;
    // Counts for positive and negative edges on the bottom sensor.
    int32_t bottom_disc_posedge_count;
    int32_t bottom_disc_negedge_count;
    // The most recent encoder position read after the most recent
    // negedge and a count of how many times it's been updated.
    double bottom_disc_negedge_wait_position;
    int32_t bottom_disc_negedge_wait_count;
    // The most recent index position at the posedge of the top disc detect
    // and a count of how many edges have been seen.
    int32_t top_disc_posedge_count;
    double top_disc_posedge_position;
    // The most recent index position at the negedge of the top disc detect
    // and a count of how many edges have been seen.
    int32_t top_disc_negedge_count;
    double top_disc_negedge_position;
  };

  message Output {
    // Intake roller(s) output voltage.
    // Positive means into the robot.
    double intake_voltage;
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
    // Number of discs seen by the hopper.
    int32_t total_disc_count;
    // Number of discs shot by the shooter.
    int32_t shot_disc_count;
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
