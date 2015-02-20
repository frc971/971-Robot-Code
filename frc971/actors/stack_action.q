package frc971.actors;

import "aos/common/actions/actions.q";

queue_group StackActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    uint32_t params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group StackActionQueueGroup stack_action;
