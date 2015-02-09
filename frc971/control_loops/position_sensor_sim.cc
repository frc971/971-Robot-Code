#include "frc971/control_loops/position_sensor_sim.h"

namespace frc971 {
namespace control_loops {

PositionSensorSimulator::PositionSensorSimulator(double offset,
                                                 double index_diff,
                                                 double pot_noise_stddev)
    : index_diff_(index_diff),
      pot_noise_(0, pot_noise_stddev) {
  OverrideParams(offset, pot_noise_stddev);
}

void PositionSensorSimulator::OverrideParams(double offset,
                                             double pot_noise_stddev) {
  cur_index_segment_ = 0;
  cur_index_ = 0;
  index_count_ = 0;
  cur_pos_ = 0;
  offset_ = offset;
  pot_noise_.set_standard_deviation(pot_noise_stddev);
}

void PositionSensorSimulator::MoveTo(double new_pos) {
  // Which index pulse we're on.
  const int new_index = static_cast<int>((new_pos + offset_) / index_diff_);

  if (new_index < cur_index_segment_) {
    // Position is decreasing.
    cur_index_ = new_index + 1;
    index_count_++;
  } else if (new_index > cur_index_segment_) {
    // Position is increasing.
    cur_index_ = new_index;
    index_count_++;
  }

  cur_index_segment_ = new_index;
  cur_pos_ = new_pos;
}

void PositionSensorSimulator::GetSensorValues(PotAndIndexPosition *values) {
  values->pot = cur_pos_ + offset_;
  values->pot = pot_noise_.AddNoiseToSample(values->pot);
  values->encoder = cur_pos_;

  if (index_count_ == 0) {
    values->latched_pot = 0.0;
    values->latched_encoder = 0.0;
  } else {
    values->latched_pot = cur_index_ * index_diff_;
    values->latched_encoder = cur_index_ * index_diff_;
  }

  values->index_pulses = index_count_;
}

}  // namespace control_loops
}  // namespace frc971
