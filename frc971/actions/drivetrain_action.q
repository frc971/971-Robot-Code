package frc971.actions;

import "aos/common/actions/actions.q";

queue_group DrivetrainActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    double left_initial_position;
    double right_initial_position;
    double y_offset;
    double theta_offset;
    double maximum_velocity;
    double maximum_acceleration;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group DrivetrainActionQueueGroup drivetrain_action;
