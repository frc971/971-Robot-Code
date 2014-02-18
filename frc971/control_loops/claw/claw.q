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

// All angles here are 0 horizontal, positive up.
queue_group ClawGroup {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // The angle of the bottom claw.
    double bottom_angle;
    // How much higher the top claw is.
    double separation_angle;
    bool intake;
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

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue aos.control_loops.Status status;
};

queue_group ClawGroup claw_queue_group;
