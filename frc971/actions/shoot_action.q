package frc971.actions;

import "frc971/actions/action.q";

queue_group ShootActionQueueGroup {
  implements frc971.actions.ActionQueueGroup;

  queue frc971.actions.Goal goal;
  queue frc971.actions.Status status;
};

queue_group ShootActionQueueGroup shoot_action;
