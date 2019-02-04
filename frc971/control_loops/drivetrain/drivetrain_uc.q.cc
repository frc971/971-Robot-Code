#include "frc971/control_loops/drivetrain/drivetrain_uc.q.h"
#include <inttypes.h>

namespace frc971 {
namespace control_loops {

GearLogging::GearLogging() { Zero(); }

void GearLogging::Zero() {
  controller_index = 0;
  left_loop_high = false;
  right_loop_high = false;
  left_state = 0;
  right_state = 0;
}

CIMLogging::CIMLogging() { Zero(); }

void CIMLogging::Zero() {
  left_in_gear = false;
  right_in_gear = false;
  left_motor_speed = 0.0f;
  right_motor_speed = 0.0f;
  left_velocity = 0.0f;
  right_velocity = 0.0f;
}

void DrivetrainQueue_Goal::Zero() {
  wheel = 0.0f;
  wheel_velocity = 0.0f;
  wheel_torque = 0.0f;
  throttle = 0.0f;
  throttle_velocity = 0.0f;
  throttle_torque = 0.0f;
  highgear = false;
  quickturn = false;
  control_loop_driving = false;
  left_goal = 0.0f;
  right_goal = 0.0f;
  max_ss_voltage = 0.0f;
  //linear.max_velocity = 0.0f;
  //linear.max_acceleration = 0.0f;
  //angular.max_velocity = 0.0f;
  //angular.max_acceleration = 0.0f;
}

DrivetrainQueue_Goal::DrivetrainQueue_Goal() { Zero(); }

void DrivetrainQueue_Position::Zero() {
  left_encoder = 0.0f;
  right_encoder = 0.0f;
  left_speed = 0.0f;
  right_speed = 0.0f;
  left_shifter_position = 0.0f;
  right_shifter_position = 0.0f;
  low_left_hall = 0.0f;
  high_left_hall = 0.0f;
  low_right_hall = 0.0f;
  high_right_hall = 0.0f;
}

DrivetrainQueue_Position::DrivetrainQueue_Position() { Zero(); }

void DrivetrainQueue_Output::Zero() {
  left_voltage = 0.0f;
  right_voltage = 0.0f;
  left_high = false;
  right_high = false;
}

DrivetrainQueue_Output::DrivetrainQueue_Output() { Zero(); }

void DrivetrainQueue_Status::Zero() {
  robot_speed = 0.0f;
  estimated_left_position = 0.0f;
  estimated_right_position = 0.0f;
  estimated_left_velocity = 0.0f;
  estimated_right_velocity = 0.0f;
  uncapped_left_voltage = 0.0f;
  uncapped_right_voltage = 0.0f;
  left_voltage_error = 0.0f;
  right_voltage_error = 0.0f;
  profiled_left_position_goal = 0.0f;
  profiled_right_position_goal = 0.0f;
  profiled_left_velocity_goal = 0.0f;
  profiled_right_velocity_goal = 0.0f;
  estimated_angular_velocity_error = 0.0f;
  estimated_heading = 0.0f;
  output_was_capped = false;
  ground_angle = 0.0f;
  gear_logging.controller_index = 0;
  gear_logging.left_loop_high = false;
  gear_logging.right_loop_high = false;
  gear_logging.left_state = 0;
  gear_logging.right_state = 0;
  cim_logging.left_in_gear = false;
  cim_logging.right_in_gear = false;
  cim_logging.left_motor_speed = 0.0f;
  cim_logging.right_motor_speed = 0.0f;
  cim_logging.left_velocity = 0.0f;
  cim_logging.right_velocity = 0.0f;
}

DrivetrainQueue_Status::DrivetrainQueue_Status() { Zero(); }

}  // namespace control_loops
}  // namespace frc971
