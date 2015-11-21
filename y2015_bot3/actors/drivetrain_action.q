package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct DrivetrainActionParams {
  double left_initial_position;
  double right_initial_position;
  double y_offset;
  double theta_offset;
  double maximum_velocity;
  double maximum_acceleration;
  double maximum_turn_velocity;
  double maximum_turn_acceleration;
};

queue_group DrivetrainActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    DrivetrainActionParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group DrivetrainActionQueueGroup drivetrain_action;
