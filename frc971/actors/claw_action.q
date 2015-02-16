package frc971.actors;

import "aos/common/actions/actions.q";

struct ClawParams {
  double claw_angle;
  double claw_max_velocity;
  // Positive is sucking in, negative is spitting out.
  double intake_voltage;
  bool rollers_closed;
};

queue_group ClawActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    ClawParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group ClawActionQueueGroup claw_action;
