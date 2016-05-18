#include "frc971/control_loops/position_sensor_sim.h"

#include <cmath>

namespace frc971 {
namespace control_loops {

/* Index pulse/segment Explanation:
 *
 * The index segments are labelled starting at zero and go up. Each index
 * segment is the space between the two bordering index pulses. The length of
 * each index segment is determined by the `index_diff` variable in the
 * constructor below.
 *
 * The index pulses are encountered when the mechanism moves from one index
 * segment to another.
 *
 *         index segment
 *               |
 *               V
 *
 * +--- 0---+--- 1---+--- 2---+--- 3---+--- 4---+--- 5---+--- 6---+
 *
 * |        |        |        |        |        |        |        |
 * 0        1        2        3        4        5        6        7
 *
 *                   A
 *                   |
 *              index pulse
 */

PositionSensorSimulator::PositionSensorSimulator(double index_diff,
                                                 unsigned int noise_seed)
    : index_diff_(index_diff), pot_noise_(noise_seed, 0.0) {
  Initialize(0.0, 0.0);
}

void PositionSensorSimulator::Initialize(double start_position,
                                         double pot_noise_stddev,
                                         double known_index_pos /* = 0*/) {
  // We're going to make the index pulse we know "segment zero".
  cur_index_segment_ = floor((start_position - known_index_pos) / index_diff_);
  known_index_pos_ = known_index_pos;
  cur_index_ = 0;
  index_count_ = 0;
  cur_pos_ = start_position;
  start_position_ = start_position;
  pot_noise_.set_standard_deviation(pot_noise_stddev);
}

void PositionSensorSimulator::MoveTo(double new_pos) {
  // Compute which index segment we're in. In other words, compute between
  // which two index pulses we are.
  const int new_index_segment =
      floor((new_pos - known_index_pos_) / index_diff_);

  if (new_index_segment < cur_index_segment_) {
    // We've crossed an index pulse in the negative direction. That means the
    // index pulse we just crossed is the higher end of the current index
    // segment. For example, if the mechanism moved from index segment four to
    // index segment three, then we just crossed index pulse 4.
    cur_index_ = new_index_segment + 1;
    index_count_++;
  } else if (new_index_segment > cur_index_segment_) {
    // We've crossed an index pulse in the positive direction. That means the
    // index pulse we just crossed is the lower end of the index segment. For
    // example, if the mechanism moved from index segment seven to index
    // segment eight, then we just crossed index pulse eight.
    cur_index_ = new_index_segment;
    index_count_++;
  }

  if (new_index_segment != cur_index_segment_) {
    latched_pot_ = pot_noise_.AddNoiseToSample(cur_index_ * index_diff_ +
                                               known_index_pos_);
  }

  cur_index_segment_ = new_index_segment;
  cur_pos_ = new_pos;
}

void PositionSensorSimulator::GetSensorValues(PotAndIndexPosition *values) {
  values->pot = pot_noise_.AddNoiseToSample(cur_pos_);
  values->encoder = cur_pos_ - start_position_;

  if (index_count_ == 0) {
    values->latched_pot = 0.0;
    values->latched_encoder = 0.0;
  } else {
    // Determine the position of the index pulse relative to absolute zero.
    double index_pulse_position = cur_index_ * index_diff_ + known_index_pos_;

    // Populate the latched pot/encoder samples.
    values->latched_pot = latched_pot_;
    values->latched_encoder = index_pulse_position - start_position_;
  }

  values->index_pulses = index_count_;
}

}  // namespace control_loops
}  // namespace frc971
