package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct StackParams {
  // Angle to move the claw to when picking up.
  double claw_out_angle;
};

queue_group StackActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    StackParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group StackActionQueueGroup stack_action;
