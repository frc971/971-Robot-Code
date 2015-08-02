package frc971.actors;

import "aos/common/actions/actions.q";

import "y2015/actors/stack_action_params.q";
import "y2015/actors/lift_action_params.q";

// Parameters to send with start.
struct StackAndLiftParams {
  // Stack parameters
  StackParams stack_params;
  // True if the fridge should be clamped while lifting.
  bool grab_after_stack;
  // The time after clamping to pause.
  double clamp_pause_time;
  // Lift parameters
  LiftParams lift_params;
  // True if the fridge should be clamped when done.
  bool grab_after_lift;
};

queue_group StackAndLiftActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    StackAndLiftParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group StackAndLiftActionQueueGroup stack_and_lift_action;
