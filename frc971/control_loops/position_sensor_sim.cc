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
 *
 *
 *
 * Absolute encoder explanation:
 *
 * If we were to graph the output of an absolute encoder that resets every 0.1
 * meters for example, it would looks something like the following. The y-axis
 * represents the output of the absolute encoder. The x-axis represents the
 * actual position of the robot's mechanism.
 *
 *          1 encoder segment
 *              +------+
 *
 *      |
 *  0.1 +      /|     /|     /|     /|     /|     /|     /|     /|
 *      |     / |    / |    / |    / |    / |    / |    / |    / |
 *      |    /  |   /  |   /  |   /  |   /  |   /  |   /  |   /  |
 *      |   /   |  /   |  /   |  /   |  /   |  /   |  /   |  /   |
 *      |  /    | /    | /    | /    | /    | /    | /    | /    |
 *      | /     |/     |/     |/     |/     |/     |/     |/     |
 *  0.0 ++------+------+------+------+------+------+------+------+----
 *      0.05   0.15   0.25   0.35   0.45   0.55   0.65   0.75   0.85
 *
 * An absolute encoder can be used to determine exactly where the mechanism in
 * question is within a certain segment. As long as you know a single combo of
 * absolute encoder reading and mechanism location you can extrapolate the
 * remainder of the graph.
 */

PositionSensorSimulator::PositionSensorSimulator(double index_diff,
                                                 unsigned int noise_seed)
    : index_diff_(index_diff), pot_noise_(noise_seed, 0.0) {
  Initialize(0.0, 0.0);
}

void PositionSensorSimulator::Initialize(
    double start_position, double pot_noise_stddev,
    double known_index_pos /* = 0*/,
    double known_absolute_encoder_pos /* = 0*/) {
  // We're going to make the index pulse we know "segment zero".
  cur_index_segment_ = floor((start_position - known_index_pos) / index_diff_);
  known_index_pos_ = known_index_pos;
  known_absolute_encoder_ = known_absolute_encoder_pos;
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

void PositionSensorSimulator::GetSensorValues(IndexPosition *values) {
  values->encoder = cur_pos_ - start_position_;

  if (index_count_ == 0) {
    values->latched_encoder = 0.0;
  } else {
    // Determine the position of the index pulse relative to absolute zero.
    double index_pulse_position = cur_index_ * index_diff_ + known_index_pos_;

    // Populate the latched encoder samples.
    values->latched_encoder = index_pulse_position - start_position_;
  }

  values->index_pulses = index_count_;
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

void PositionSensorSimulator::GetSensorValues(PotAndAbsolutePosition *values) {
  values->pot = pot_noise_.AddNoiseToSample(cur_pos_);
  values->encoder = cur_pos_ - start_position_;
  // TODO(phil): Create some lag here since this is a PWM signal it won't be
  // instantaneous like the other signals. Better yet, its lag varies
  // randomly with the distribution varying depending on the reading.
  values->absolute_encoder =
      ::std::remainder(cur_pos_ + known_absolute_encoder_, index_diff_);
  if (values->absolute_encoder < 0) {
    values->absolute_encoder += index_diff_;
  }
}

}  // namespace control_loops
}  // namespace frc971
