package y2016.actors;

import "aos/common/actions/actions.q";

struct AutonomousActionParams {
  int32_t mode;  // 0 = single ball auto
};

queue_group AutonomousActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    AutonomousActionParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group AutonomousActionQueueGroup autonomous_action;
