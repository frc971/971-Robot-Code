package y2015.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct PickupParams {
  // Angle to pull in at the top.
  double pickup_angle;
  // Angle to start sucking in above.
  double suck_angle;
  // Angle to finish sucking at.
  double suck_angle_finish;
  // Angle to go to once we get to the top to finish pulling in.
  double pickup_finish_angle;
  // Power to pull the bin in at the top.
  double intake_voltage;
  // Time to pull the bin in for.
  double intake_time;
};

queue_group PickupActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    PickupParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group PickupActionQueueGroup pickup_action;
