package y2015.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
// This action picks a horizontal can from the claw.
struct HorizontalCanPickupParams {
  // Elevator catch height.
  double elevator_height;
  // Angle to move the claw to when placing the base of the can on the robot.
  double pickup_angle;

  // Time and power to spit the can out before lifting.
  double spit_time;
  double spit_power;

  // Time and power to pull the can in when lifted.
  double suck_time;
  double suck_power;

  // Time to push down and suck in to slide the claw down on the can.
  double claw_settle_time;
  double claw_settle_power;

  // Angle to lift the claw to to lift the can.
  double claw_full_lift_angle;

  // Angle to move the claw back down to.
  double claw_end_angle;

  // The end arm and elevator position once we are done lifting.
  double elevator_end_height;
  double arm_end_angle;
};

queue_group HorizontalCanPickupActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    HorizontalCanPickupParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group HorizontalCanPickupActionQueueGroup horizontal_can_pickup_action;
