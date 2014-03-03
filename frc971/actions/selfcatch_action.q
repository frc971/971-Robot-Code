package frc971.actions;

queue_group SelfCatchActionGroup {
  message Status {
    bool running;
  };

  message Goal {
    // If true, run this action.  If false, cancel the action if it is
    // currently running.
    bool run; // Shot power in joules.
  };

  queue Goal goal;
  queue Status status;
};

queue_group SelfCatchActionGroup selfcatch_action;
