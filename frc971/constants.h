#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_

namespace frc971 {
namespace constants {

struct ZeroingConstants {
  // The number of samples in the moving average filter.
  int average_filter_size;
  // The difference in scaled units between two index pulses.
  double index_difference;
  // The absolute position in scaled units of one of the index pulses.
  double measured_index_position;
  // Value between 0 and 1 which determines a fraction of the index_diff
  // you want to use.
  double allowable_encoder_error;
};

struct EncoderPlusIndexZeroingConstants {
  // The amount of index pulses in the limb's range of motion.
  int num_index_pulses;
};

struct PotAndAbsoluteEncoderZeroingConstants {
  // The distance that the absolute encoder needs to complete a full rotation.
  double abs_duration;
  // Sample mechanism angle and absolute encoder value.
  double sample_abs_value;
  double sample_degrees;
};

// Defines a range of motion for a subsystem.
// These are all absolute positions in scaled units.
struct Range {
  double lower_hard;
  double upper_hard;
  double lower;
  double upper;
};

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
