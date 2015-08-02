package frc971.actors;

import "aos/common/actions/actions.q";
import "y2015/actors/lift_action_params.q";

// Parameters to send with start.
struct HeldToLiftParams {
  // The maximum claw value to avoid collisions.
  double claw_out_angle;

  // The value to move the arm forwards to clear the stack when lowering.
  double arm_clearance;
  // End height.
  double bottom_height;
  // Amount to wait for the elevator to settle before lifting.
  double before_lift_settle_time;
  // Amount to wait to clamp.
  double clamp_pause_time;

  // Lift parameters
  LiftParams lift_params;
};

queue_group HeldToLiftActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    HeldToLiftParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group HeldToLiftActionQueueGroup held_to_lift_action;
