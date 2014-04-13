package frc971.actions;

import "frc971/actions/action.q";

queue_group DrivetrainActionQueueGroup {
  implements frc971.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    double left_initial_position;
    double right_initial_position;
    double y_offset;
    double maximum_velocity;
  };

  queue Goal goal;
  queue frc971.actions.Status status;
};

queue_group DrivetrainActionQueueGroup drivetrain_action;
