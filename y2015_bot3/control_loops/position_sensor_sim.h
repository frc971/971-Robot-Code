#ifndef Y2015_BOT3_CONTROL_LOOPS_POSITION_SENSOR_SIM_H_
#define Y2015_BOT3_CONTROL_LOOPS_POSITION_SENSOR_SIM_H_

#include "y2015_bot3/control_loops/elevator/elevator.q.h"

namespace y2015_bot3 {
namespace control_loops {

// NOTE: All encoder values in this class are assumed to be in
// translated SI units.

class PositionSensorSimulator {
 public:

  // Set new parameters for the sensors. This is useful for unit tests to change
  // the simulated sensors' behavior on the fly.
  // start_position: The position relative to absolute zero where the simulated
  //                 structure starts. For example, to simulate the elevator
  //                 starting at 40cm above absolute zero, set this to 0.4.
  void Initialize(double start_position, double hall_effect_position);

  // Simulate the structure moving to a new position. The new value is measured
  // relative to absolute zero. This will update the simulated sensors with new
  // readings.
  // new_position: The new position relative to absolute zero.
  void MoveTo(double new_position);

  // Get the current values of the simulated sensors.
  // values: The target structure will be populated with simulated sensor
  //         readings. The readings will be in SI units. For example the units
  //         can be given in radians, meters, etc.
  void GetSensorValues(control_loops::ElevatorQueue::Position* position);

 private:
  // the position of the bottom hall effect sensor.
  double hall_effect_position_;
  // Current position of the mechanism relative to absolute zero.
  double cur_pos_;
  // Starting position of the mechanism relative to absolute zero. See the
  // `starting_position` parameter in the constructor for more info.
  double start_position_;
};

}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_POSITION_SENSOR_SIM_H_
