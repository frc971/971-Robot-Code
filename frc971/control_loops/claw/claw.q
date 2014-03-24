package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";
import "frc971/control_loops/control_loops.q";

struct HalfClawPosition {
  // The current position of this half of the claw.
  double position;

  // The hall effect sensor at the front limit.
  HallEffectStruct front;
  // The hall effect sensor in the middle to use for real calibration.
  HallEffectStruct calibration;
  // The hall effect at the back limit.
  HallEffectStruct back;

  // The encoder value at the last posedge of any of the claw hall effect
  // sensors (front, calibration, or back).
  double posedge_value;
  // The encoder value at the last negedge of any of the claw hall effect
  // sensors (front, calibration, or back).
  double negedge_value;
};

// All angles here are 0 vertical, positive "up" (aka backwards).
queue_group ClawGroup {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // The angle of the bottom claw.
    double bottom_angle;
    // How much higher the top claw is.
    double separation_angle;
    // top claw intake roller
    double intake;
    // bottom claw tusk centering
    double centering;
  };

  message Position {
    // All the top claw information.
    HalfClawPosition top;
    // All the bottom claw information.
    HalfClawPosition bottom;
  };

  message Output {
    double intake_voltage;
    double top_claw_voltage;
    double bottom_claw_voltage;
    double tusk_voltage;
  };

  message Status {
    // True if zeroed enough for the current period (autonomous or teleop).
    bool zeroed;
    // True if zeroed as much as we will force during autonomous.
    bool zeroed_for_auto;
    // True if zeroed and within tolerance for separation and bottom angle.
    bool done;
    // True if zeroed and within tolerance for separation and bottom angle.
    // seperation allowance much wider as a ball may be included
    bool done_with_ball;
    // Dump the values of the state matrix.
    double bottom;
    double bottom_velocity;
    double separation;
    double separation_velocity;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ClawGroup claw_queue_group;

struct ClawPositionToLog {
	double top;
	double bottom;
};

struct ClawGoalToLog {
	double bottom_pos;
	double bottom_vel;
	double separation;
};
