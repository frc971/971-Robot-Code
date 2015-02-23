package frc971.actors;

import "aos/common/actions/actions.q";

// Parameters to send with start.
struct FridgeProfileParams {
  double arm_angle;
  double arm_max_velocity;
  double arm_max_acceleration;
  double elevator_height;
  double elevator_max_velocity;
  double elevator_max_acceleration;
  bool top_front_grabber;
  bool top_back_grabber;
  bool bottom_front_grabber;
  bool bottom_back_grabber;
};

// Debug errors.
struct ErrorToLog {
  float arm_error;
  float profile_error_arm;
  float elevator_error;
  float profile_error_elevator;
};

queue_group FridgeProfileActionQueueGroup {
  implements aos.common.actions.ActionQueueGroup;

  message Goal {
    uint32_t run;
    FridgeProfileParams params;
  };

  queue Goal goal;
  queue aos.common.actions.Status status;
};

queue_group FridgeProfileActionQueueGroup fridge_profile_action;
