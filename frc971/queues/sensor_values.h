#ifndef FRC971_QUEUES_SENSOR_VALUES_H_
#define FRC971_QUEUES_SENSOR_VALUES_H_

#include <stdint.h>

namespace frc971 {

struct sensor_values {
  // Anonymous union to make fixing the byte order on all of the 4-byte long
  // values easier.
  // TODO(brians) name this better
  union {
    struct {
      int32_t drive_left_encoder, drive_right_encoder;
      int32_t shooter_encoder;
      int32_t index_encoder, bottom_disc_negedge_wait_position;
      int32_t bottom_disc_posedge_count, bottom_disc_negedge_count;
      int32_t bottom_disc_negedge_wait_count;
      int32_t top_disc_posedge_count, top_disc_negedge_count;
      int32_t top_disc_posedge_position, top_disc_negedge_position;
      int32_t wrist_position, wrist_edge_position;
      int32_t angle_adjust_position;
      int32_t angle_adjust_middle_edge_position;
      int32_t angle_adjust_bottom_edge_position;
    };
    uint32_t encoders[17];
  };

  bool wrist_hall_effect;
  bool angle_adjust_middle_hall_effect;
  bool angle_adjust_bottom_hall_effect;

  bool top_disc, bottom_disc;
};

}  // namespace frc971

#endif  // FRC971_QUEUES_SENSOR_VALUES_H_
