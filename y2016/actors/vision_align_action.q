package y2016.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct VisionAlignActionParams {
  int32_t run;
};

queue_group VisionAlignActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    VisionAlignActionParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group VisionAlignActionQueueGroup vision_align_action;
