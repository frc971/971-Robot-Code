package frc971.actions;

queue_group DrivetrainActionQueueGroup {
  message Status {
    bool running;
  };

  message Goal {
    // If true, run this action.  If false, cancel the action if it is
    // currently running.
    bool run;
    double left_initial_position;
    double right_initial_position;
    double y_offset;
    double maximum_velocity;
  };

  queue Goal goal;
  queue Status status;
};

queue_group DrivetrainActionQueueGroup drivetrain_action;
