package frc971.autonomous;

import "aos/common/actions/actions.q";

message AutoControl {
  // True if auto mode should be running, false otherwise.
  bool run_auto;
};

queue AutoControl autonomous;

message AutonomousMode {
  // Mode read from the mode setting sensors.
  int32_t mode;
};

queue AutonomousMode auto_mode;

struct AutonomousActionParams {
  // The mode from the sensors when auto starts.
  int32_t mode;
};

queue_group AutonomousActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    AutonomousActionParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group AutonomousActionQueueGroup autonomous_action;
