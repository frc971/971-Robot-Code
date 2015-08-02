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

}  // namespace constants
}  // namespace frc971

#endif  // FRC971_CONSTANTS_H_
