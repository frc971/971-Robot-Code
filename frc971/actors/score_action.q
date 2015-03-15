package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct ScoreParams {
  // Height for the upper fridge pivot to be when we're scoring.
  double height;
  // If false, then extend. Otherwise, place the stack and retract.
  bool place_the_stack;
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
