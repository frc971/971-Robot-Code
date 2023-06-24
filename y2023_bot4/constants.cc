#include "y2023_bot4/constants.h"

#include <cstdint>

#include "glog/logging.h"

#include "aos/network/team_number.h"

namespace y2023_bot4 {
namespace constants {
Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;
  Values r;
  auto *const front_left_zeroing_constants = &r.front_left_zeroing_constants;
  auto *const front_right_zeroing_constants = &r.front_right_zeroing_constants;
  auto *const back_left_zeroing_constants = &r.back_left_zeroing_constants;
  auto *const back_right_zeroing_constants = &r.back_right_zeroing_constants;

  front_left_zeroing_constants->average_filter_size = 0;
  front_left_zeroing_constants->one_revolution_distance = 2 * M_PI;
  front_left_zeroing_constants->measured_absolute_position = 0.76761395509829;
  front_left_zeroing_constants->zeroing_threshold = 0.0;
  front_left_zeroing_constants->moving_buffer_size = 0.0;
  front_left_zeroing_constants->allowable_encoder_error = 0.0;

  front_right_zeroing_constants->average_filter_size = 0;
  front_right_zeroing_constants->one_revolution_distance = 2 * M_PI;
  front_right_zeroing_constants->measured_absolute_position = 0.779403958443922;
  front_right_zeroing_constants->zeroing_threshold = 0.0;
  front_right_zeroing_constants->moving_buffer_size = 0.0;
  front_right_zeroing_constants->allowable_encoder_error = 0.0;

  back_left_zeroing_constants->average_filter_size = 0;
  back_left_zeroing_constants->one_revolution_distance = 2 * M_PI;
  back_left_zeroing_constants->measured_absolute_position = 0.053439698061417;
  back_left_zeroing_constants->zeroing_threshold = 0.0;
  back_left_zeroing_constants->moving_buffer_size = 0.0;
  back_left_zeroing_constants->allowable_encoder_error = 0.0;

  back_right_zeroing_constants->average_filter_size = 0;
  back_right_zeroing_constants->one_revolution_distance = 2 * M_PI;
  back_right_zeroing_constants->measured_absolute_position = 0.719329333121509;
  back_right_zeroing_constants->zeroing_threshold = 0.0;
  back_right_zeroing_constants->moving_buffer_size = 0.0;
  back_right_zeroing_constants->allowable_encoder_error = 0.0;

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }
}  // namespace constants
}  // namespace y2023_bot4
