package y2015.actors;

import "aos/common/actions/actions.q";
import "y2015/actors/lift_action_params.q";

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
