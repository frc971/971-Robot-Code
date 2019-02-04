#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_Q_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_Q_H_
#include <array>
#include "aos/macros.h"

namespace frc971 {
namespace control_loops {

struct GearLogging {
  GearLogging();
  void Zero();

  int8_t controller_index;
  bool left_loop_high;
  bool right_loop_high;
  int8_t left_state;
  int8_t right_state;
};

struct CIMLogging {
  CIMLogging();
  void Zero();

  bool left_in_gear;
  bool right_in_gear;
  float left_motor_speed;
  float right_motor_speed;
  float left_velocity;
  float right_velocity;
};

struct DrivetrainQueue_Goal {
  DrivetrainQueue_Goal();
  void Zero();

  float wheel;
  float wheel_velocity;
  float wheel_torque;
  float throttle;
  float throttle_velocity;
  float throttle_torque;
  bool highgear;
  bool quickturn;
  bool control_loop_driving;
  float left_goal;
  float right_goal;
  float max_ss_voltage;
};

struct DrivetrainQueue_Position {
  DrivetrainQueue_Position();
  void Zero();

  float left_encoder;
  float right_encoder;
  float left_speed;
  float right_speed;
  float left_shifter_position;
  float right_shifter_position;
  float low_left_hall;
  float high_left_hall;
  float low_right_hall;
  float high_right_hall;
};

struct DrivetrainQueue_Output {
  DrivetrainQueue_Output();
  void Zero();

  float left_voltage;
  float right_voltage;
  bool left_high;
  bool right_high;
};

struct DrivetrainQueue_Status {
  DrivetrainQueue_Status();
  void Zero();

  float robot_speed;
  float estimated_left_position;
  float estimated_right_position;
  float estimated_left_velocity;
  float estimated_right_velocity;
  float uncapped_left_voltage;
  float uncapped_right_voltage;
  float left_voltage_error;
  float right_voltage_error;
  float profiled_left_position_goal;
  float profiled_right_position_goal;
  float profiled_left_velocity_goal;
  float profiled_right_velocity_goal;
  float estimated_angular_velocity_error;
  float estimated_heading;
  bool output_was_capped;
  float ground_angle;
  ::frc971::control_loops::GearLogging gear_logging;
  ::frc971::control_loops::CIMLogging cim_logging;
};

class DrivetrainQueue {
 public:
  typedef DrivetrainQueue_Goal Goal;
  DrivetrainQueue_Goal goal;
  typedef DrivetrainQueue_Position Position;
  DrivetrainQueue_Position position;
  typedef DrivetrainQueue_Output Output;
  DrivetrainQueue_Output output;
  typedef DrivetrainQueue_Status Status;
  DrivetrainQueue_Status status;
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_Q_H_
