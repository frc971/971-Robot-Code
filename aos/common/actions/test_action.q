package aos.common.actions;

import "aos/common/actions/actions.q";

queue_group TestActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    uint32_t params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

struct MyParams {
  double param1;
  int32_t param2;
};

queue_group TestAction2QueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    MyParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group TestActionQueueGroup test_action;
queue_group TestAction2QueueGroup test_action2;
