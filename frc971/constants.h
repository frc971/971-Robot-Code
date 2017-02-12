#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_

#include <cstddef>

namespace frc971 {
namespace constants {

struct PotAndIndexPulseZeroingConstants {
  // The number of samples in the moving average filter.
  size_t average_filter_size;
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
  // The number of samples in the moving average filter.
  size_t average_filter_size;
  // The distance that the absolute encoder needs to complete a full rotation.
  double one_revolution_distance;
  // Measured absolute position of the encoder when at zero.
  double measured_absolute_position;

  // Treshold for deciding if we are moving
  // TODO(austin): Figure out what this is actually measuring.
  double zeroing_threshold;
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
