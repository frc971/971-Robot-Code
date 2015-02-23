#ifndef FRC971_CONTROL_LOOPS_POSITION_SENSOR_SIM_H_
#define FRC971_CONTROL_LOOPS_POSITION_SENSOR_SIM_H_

#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/gaussian_noise.h"

namespace frc971 {
namespace control_loops {

// NOTE: All potentiometer and encoder values in this class are assumed to be in
// translated SI units.

class PositionSensorSimulator {
 public:
  // index_diff: The interval between index pulses. This is measured in SI
  //             units. For example, if an index pulse hits every 5cm on the
  //             elevator, set this to 0.05.
  // noise_seed: The seed to feed into the random number generator for the
  //             potentiometer values.
  // TODO(danielp): Allow for starting with a non-zero encoder value.
  PositionSensorSimulator(double index_diff, unsigned int noise_seed = 0);

  // Set new parameters for the sensors. This is useful for unit tests to change
  // the simulated sensors' behavior on the fly.
  // start_position: The position relative to absolute zero where the simulated
  //                 structure starts. For example, to simulate the elevator
  //                 starting at 40cm above absolute zero, set this to 0.4.
  // pot_noise_stddev: The pot noise is sampled from a gaussian distribution.
  //                   This specifies the standard deviation of that
  //                   distribution.
  // known_index_pos: The absolute position of an index pulse.
  void Initialize(double start_position,
                  double pot_noise_stddev,
                  double known_index_pos = 0.0);

  // Simulate the structure moving to a new position. The new value is measured
  // relative to absolute zero. This will update the simulated sensors with new
  // readings.
  // new_position: The new position relative to absolute zero.
  void MoveTo(double new_position);

  // Get the current values of the simulated sensors.
  // values: The target structure will be populated with simulated sensor
  //         readings. The readings will be in SI units. For example the units
  //         can be given in radians, meters, etc.
  void GetSensorValues(PotAndIndexPosition* values);

 private:
  // The absolute segment between two index pulses the simulation is on. For
  // example, when the current position is betwen index pulse zero and one,
  // the current index segment is considered to be zero. Index segment one is
  // between index pulses 1 and 2, etc.
  int cur_index_segment_;
  // Index pulse to use for calculating latched sensor values, relative to
  // absolute zero. In other words this always holds the index pulse that was
  // encountered most recently.
  int cur_index_;
  // How many index pulses we've seen.
  int index_count_;
  // Distance between index pulses on the mechanism.
  double index_diff_;
  // Absolute position of a known index pulse.
  double known_index_pos_;
  // Current position of the mechanism relative to absolute zero.
  double cur_pos_;
  // Starting position of the mechanism relative to absolute zero. See the
  // `starting_position` parameter in the constructor for more info.
  double start_position_;
  // Gaussian noise to add to pot readings.
  GaussianNoise pot_noise_;
};

}  // namespace control_loops
}  // namespace frc971

#endif /* FRC971_CONTROL_LOOPS_POSITION_SENSOR_SIM_H_ */
