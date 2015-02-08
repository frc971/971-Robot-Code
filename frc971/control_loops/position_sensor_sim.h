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
  // offset: The difference between zero on the pot and an index pulse.
  // index_diff: The interval between index pulses.
  // pot_noise_stddev: The pot noise is sampled from a gaussian distribution.
  //                   This specifies the standard deviation of that
  //                   distribution.
  // TODO(danielp): Allow for starting with a non-zero encoder value.
  // TODO(danielp): Allow for the first index pulse to be at a non-zero
  // position.
  PositionSensorSimulator(double offset, double index_diff,
                          double pot_noise_stddev);

  // Set new parameters for the sensors. This is useful for unit tests to change
  // the simulated sensors' behavior on the fly.
  void OverrideParams(double start_pos, double pot_noise_stddev);

  // Change sensors to a new position.
  // new_pos: The new position. (This is an absolute position.)
  void MoveTo(double new_pos);

  // Get the current values of the sensors.
  // values: These will be filled in.
  void GetSensorValues(PotAndIndexPosition* values);

 private:
  // Which index pulse we are currently on relative to the one we started on.
  int cur_index_segment_;
  // Index pulse to use for calculating latched sensor values, relative to the
  // one we started on.
  int cur_index_;
  // How many index pulses we've seen.
  int index_count_;
  // Distance between index pulses on the mechanism.
  double index_diff_;
  // Current encoder value.
  double cur_pos_;
  // Difference between zero on the pot and zero on the encoder.
  double offset_;
  // Gaussian noise to add to pot readings.
  GaussianNoise pot_noise_;
};

}  // namespace control_loops
}  // namespace frc971

#endif /* FRC971_CONTROL_LOOPS_POSITION_SENSOR_SIM_H_ */
