package y2014.actors;

import "aos/common/actions/actions.q";

queue_group ShootActionQueueGroup {
  implements frc971.actions.ActionQueueGroup;

  queue aos.common.actions.Goal goal;
  queue aos.common.actions.Status status;
};

queue_group ShootActionQueueGroup shoot_action;
