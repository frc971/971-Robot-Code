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

PositionSensorSimulator::PositionSensorSimulator(double index_difference,
                                                 unsigned int noise_seed)
    : lower_index_edge_(index_difference, noise_seed),
      upper_index_edge_(index_difference, noise_seed),
      index_difference_(index_difference) {
  Initialize(0.0, 0.0);
}

void PositionSensorSimulator::Initialize(
    double start_position, double pot_noise_stddev,
    double known_index_position /* = 0*/,
    double known_absolute_encoder_pos /* = 0*/) {
  InitializeHallEffectAndPosition(start_position, known_index_position,
                                  known_index_position);

  known_absolute_encoder_ = known_absolute_encoder_pos;

  lower_index_edge_.mutable_pot_noise()->set_standard_deviation(pot_noise_stddev);
  upper_index_edge_.mutable_pot_noise()->set_standard_deviation(pot_noise_stddev);
}

void PositionSensorSimulator::InitializeHallEffectAndPosition(
    double start_position, double known_index_lower, double known_index_upper) {
  current_position_ = start_position;
  start_position_ = start_position;

  lower_index_edge_.Initialize(start_position, known_index_lower);
  upper_index_edge_.Initialize(start_position, known_index_upper);

  posedge_count_ = 0;
  negedge_count_ = 0;
  posedge_value_ = start_position;
  negedge_value_ = start_position;
}

void PositionSensorSimulator::MoveTo(double new_position) {
  {
    const int lower_start_segment = lower_index_edge_.current_index_segment();
    lower_index_edge_.MoveTo(new_position);
    const int lower_end_segment = lower_index_edge_.current_index_segment();
    if (lower_end_segment > lower_start_segment) {
      // Moving up past the lower edge.
      ++posedge_count_;
      posedge_value_ = lower_index_edge_.IndexPulsePosition();
    }
    if (lower_end_segment < lower_start_segment) {
      // Moved down.
      ++negedge_count_;
      negedge_value_ = lower_index_edge_.IndexPulsePosition();
    }
  }

  {
    const int upper_start_segment = upper_index_edge_.current_index_segment();
    upper_index_edge_.MoveTo(new_position);
    const int upper_end_segment = upper_index_edge_.current_index_segment();
    if (upper_end_segment > upper_start_segment) {
      // Moving up past the upper edge.
      ++negedge_count_;
      negedge_value_ = upper_index_edge_.IndexPulsePosition();
    }
    if (upper_end_segment < upper_start_segment) {
      // Moved down.
      ++posedge_count_;
      posedge_value_ = upper_index_edge_.IndexPulsePosition();
    }
  }

  current_position_ = new_position;
}

void PositionSensorSimulator::GetSensorValues(IndexPosition *values) {
  values->encoder = current_position_ - start_position_;

  values->index_pulses = lower_index_edge_.index_count();
  if (values->index_pulses == 0) {
    values->latched_encoder = 0.0;
  } else {
    // Populate the latched encoder samples.
    values->latched_encoder =
        lower_index_edge_.IndexPulsePosition() - start_position_;
  }
}

void PositionSensorSimulator::GetSensorValues(PotAndIndexPosition *values) {
  values->pot = lower_index_edge_.mutable_pot_noise()->AddNoiseToSample(
      current_position_);
  values->encoder = current_position_ - start_position_;

  if (lower_index_edge_.index_count() == 0) {
    values->latched_pot = 0.0;
    values->latched_encoder = 0.0;
  } else {
    // Populate the latched pot/encoder samples.
    values->latched_pot = lower_index_edge_.latched_pot();
    values->latched_encoder =
        lower_index_edge_.IndexPulsePosition() - start_position_;
  }

  values->index_pulses = lower_index_edge_.index_count();
}

void PositionSensorSimulator::GetSensorValues(PotAndAbsolutePosition *values) {
  values->pot = lower_index_edge_.mutable_pot_noise()->AddNoiseToSample(
      current_position_);
  values->encoder = current_position_ - start_position_;
  // TODO(phil): Create some lag here since this is a PWM signal it won't be
  // instantaneous like the other signals. Better yet, its lag varies
  // randomly with the distribution varying depending on the reading.
  values->absolute_encoder = ::std::remainder(
      current_position_ + known_absolute_encoder_, index_difference_);
  if (values->absolute_encoder < 0) {
    values->absolute_encoder += index_difference_;
  }
}

void PositionSensorSimulator::GetSensorValues(HallEffectAndPosition *values) {
  values->current = lower_index_edge_.current_index_segment() !=
                    upper_index_edge_.current_index_segment();
  values->position = current_position_ - start_position_;

  values->posedge_count = posedge_count_;
  values->negedge_count = negedge_count_;
  values->posedge_value = posedge_value_ - start_position_;
  values->negedge_value = negedge_value_ - start_position_;
}

}  // namespace control_loops
}  // namespace frc971
