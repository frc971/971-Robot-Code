package frc971.actors;

import "aos/common/actions/actions.q";

struct IntakeParams {
  // If grab is true, we're intaking the tote.
  // If it is false, we're pulling the claw up and pushing the tote backwards
  // into the robot. If we're trying to intake from the HP, grab need never be
  // false. If we're trying to intake from the ground, however, the action
  // should be run twice: First, with grab true in order to grab the tote, and
  // then again with grab false in order to send it back into the robot.
  bool grab;
  // If ground is true, we're intaking from the ground. Otherwise, we'e intaking
  // from the HP.
  bool ground;
};

queue_group IntakeActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    IntakeParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group IntakeActionQueueGroup intake_action;
