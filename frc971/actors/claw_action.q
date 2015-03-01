package frc971.actors;

import "aos/common/actions/actions.q";

struct ClawParams {
  // Goal angle.
  double angle;
  // Maximum velocity during the motion.
  double max_velocity;
  // Maximum acceleration during the motion.
  double max_acceleration;
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
