package frc971.control_loops;

import "frc971/control_loops/control_loops.q";

struct ProfiledJointStatus {
  // Is the subsystem zeroed?
  bool zeroed;

  // The state of the subsystem, if applicable.  -1 otherwise.
  int32_t state;

  // If true, we have aborted.
  bool estopped;

  // Position of the joint.
  float position;
  // Velocity of the joint in units/second.
  float velocity;
  // Profiled goal position of the joint.
  float goal_position;
  // Profiled goal velocity of the joint in units/second.
  float goal_velocity;
  // Unprofiled goal position from absoulte zero  of the joint.
  float unprofiled_goal_position;
  // Unprofiled goal velocity of the joint in units/second.
  float unprofiled_goal_velocity;

  // The estimated voltage error.
  float voltage_error;

  // The calculated velocity with delta x/delta t
  float calculated_velocity;

  // Components of the control loop output
  float position_power;
  float velocity_power;
  float feedforwards_power;

  // State of the estimator.
  .frc971.EstimatorState estimator_state;
};

struct AbsoluteProfiledJointStatus {
  // Is the subsystem zeroed?
  bool zeroed;

  // The state of the subsystem, if applicable.  -1 otherwise.
  int32_t state;

  // If true, we have aborted.
  bool estopped;

  // Position of the joint.
  float position;
  // Velocity of the joint in units/second.
  float velocity;
  // Profiled goal position of the joint.
  float goal_position;
  // Profiled goal velocity of the joint in units/second.
  float goal_velocity;
  // Unprofiled goal position from absoulte zero of the joint.
  float unprofiled_goal_position;
  // Unprofiled goal velocity of the joint in units/second.
  float unprofiled_goal_velocity;

  // The estimated voltage error.
  float voltage_error;

  // The calculated velocity with delta x/delta t
  float calculated_velocity;

  // Components of the control loop output
  float position_power;
  float velocity_power;
  float feedforwards_power;

  // State of the estimator.
  .frc971.AbsoluteEstimatorState estimator_state;
};

struct IndexProfiledJointStatus {
  // Is the subsystem zeroed?
  bool zeroed;

  // The state of the subsystem, if applicable.  -1 otherwise.
  int32_t state;

  // If true, we have aborted.
  bool estopped;

  // Position of the joint.
  float position;
  // Velocity of the joint in units/second.
  float velocity;
  // Profiled goal position of the joint.
  float goal_position;
  // Profiled goal velocity of the joint in units/second.
  float goal_velocity;
  // Unprofiled goal position from absoulte zero of the joint.
  float unprofiled_goal_position;
  // Unprofiled goal velocity of the joint in units/second.
  float unprofiled_goal_velocity;

  // The estimated voltage error.
  float voltage_error;

  // The calculated velocity with delta x/delta t
  float calculated_velocity;

  // Components of the control loop output
  float position_power;
  float velocity_power;
  float feedforwards_power;

  // State of the estimator.
  .frc971.IndexEstimatorState estimator_state;
};
