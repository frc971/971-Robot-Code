package aos.common.actions;

import "aos/common/actions/actions.q";

queue_group TestActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    double test_value;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group TestActionQueueGroup test_action;
