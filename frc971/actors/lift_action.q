package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct LiftParams {
  // Lift height
  double lift_height;
  // Arm goal.
  double lift_arm;
};

queue_group LiftActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    LiftParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group LiftActionQueueGroup lift_action;
