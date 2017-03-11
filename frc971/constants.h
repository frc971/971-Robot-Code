#ifndef FRC971_CONSTANTS_H_
#define FRC971_CONSTANTS_H_

#include <cstddef>

namespace frc971 {
namespace constants {

struct HallEffectZeroingConstants {
  // The absolute position of the lower edge of the hall effect sensor.
  double lower_hall_position;
  // The absolute position of the upper edge of the hall effect sensor.
  double upper_hall_position;
  // The difference in scaled units between two hall effect edges.  This is the
  // number of units/cycle.
  double index_difference;
  // Number of cycles we need to see the hall effect high.
  size_t hall_trigger_zeroing_length;
  // Direction the system must be moving in order to zero. True is positive,
  // False is negative direction.
  bool zeroing_move_direction;
};

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
  // The amount of index pulses in the joint's range of motion.
  int index_pulse_count;
  // The difference in scaled units between two index pulses.
  double index_difference;
  // The absolute position in scaled units of one of the index pulses.
  double measured_index_position;
  // The index pulse that is known, going from lowest in the range of motion to
  // highest (Starting at 0).
  int known_index_pulse;
};

struct PotAndAbsoluteEncoderZeroingConstants {
  // The number of samples in the moving average filter.
  size_t average_filter_size;
  // The distance that the absolute encoder needs to complete a full rotation.
  double one_revolution_distance;
  // Measured absolute position of the encoder when at zero.
  double measured_absolute_position;

  // Treshold for deciding if we are moving. moving_buffer_size samples need to
  // be within this distance of each other before we use the middle one to zero.
  double zeroing_threshold;
  // Buffer size for deciding if we are moving.
  size_t moving_buffer_size;

  // Value between 0 and 1 indicating what fraction of one_revolution_distance
  // it is acceptable for the offset to move.
  double allowable_encoder_error;
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
