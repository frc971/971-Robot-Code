package y2016.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct SuperstructureActionParams {
  double partial_angle;
  double delay_time;
  double full_angle;
  double shooter_angle;
};

queue_group SuperstructureActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    SuperstructureActionParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group SuperstructureActionQueueGroup superstructure_action;
