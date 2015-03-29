package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct ScoreParams {
  // If true, move the stack first.
  bool move_the_stack;
  // If true, place the stack (possibly after moving it).
  bool place_the_stack;

  // TODO(Brian): Comments (by somebody who knows what these all mean).
  double upper_move_height;
  double begin_horizontal_move_height;
  double horizontal_move_target;
  double horizontal_start_lowering;
  double place_height;
  double home_lift_horizontal_start_position;
  double home_return_height;
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
