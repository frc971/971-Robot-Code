package frc971.autonomous;

import "aos/actions/actions.q";

// Published on ".frc971.autonomous.auto_mode"
message AutonomousMode {
  // Mode read from the mode setting sensors.
  int32_t mode;
};

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
