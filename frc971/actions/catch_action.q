package frc971.actions;

import "frc971/actions/action.q";

queue_group CatchActionGroup {
  implements frc971.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    double catch_angle;
  };

  queue Goal goal;
  queue frc971.actions.Status status;
};

queue_group CatchActionGroup catch_action;
