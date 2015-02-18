package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct ScoreParams {
  // Height for the elevator to be when we're scoring.
  double height;
};

queue_group ScoreActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    ScoreParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group ScoreActionQueueGroup score_action;
