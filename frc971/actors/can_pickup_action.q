package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
// This action picks a right side up can from the claw.
// It starts by lifting up, then moving the arm out, then lifting the can out of
// the claw, and then ends with the can inside the bot.
struct CanPickupParams {
  // Angle to move the claw to when picking up.
  double pickup_angle;
  // Height to move the elevator to when picking up.
  double pickup_height;
  // Height to move the elevator to to lift the can out of the claw.
  double lift_height;

  // End position with the can.
  double end_height;
  double end_angle;
};

queue_group CanPickupActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    CanPickupParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group CanPickupActionQueueGroup can_pickup_action;
