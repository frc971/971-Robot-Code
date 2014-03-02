package frc971.actions;

queue_group SelfCatchActionGroup {
  message Status {
    bool running;
  };

  message Goal {
    // If true, run this action.  If false, cancel the action if it is
    // currently running.
    bool run;
    double shot_angle;
  };

  queue Goal goal;
  queue Status status;
};

queue_group SelfCatchActionGroup selfcatch_action;
