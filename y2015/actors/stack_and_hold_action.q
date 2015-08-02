package frc971.actors;

import "aos/common/actions/actions.q";
import "y2015/actors/stack_action_params.q";

// Parameters to send with start.
struct StackAndHoldParams {
  // If true, there is no tote on the tray, and we should place instead.
  bool place_not_stack;

  double claw_out_angle;
  // The height just above the box to move before lowering.
  double over_box_before_place_height;

  // Bottom position.
  double bottom;

  // If we are placing, clamp the stack with the claw.
  double claw_clamp_angle;

  // Amount to wait to release.
  double clamp_pause_time;

  // The value to move the arm forwards to clear the stack when lifting.
  double arm_clearance;
  // Hold height
  double hold_height;
};

queue_group StackAndHoldActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    StackAndHoldParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group StackAndHoldActionQueueGroup stack_and_hold_action;
