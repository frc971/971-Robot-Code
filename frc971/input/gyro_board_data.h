#ifndef FRC971_INPUT_GYRO_BOARD_DATA_H_
#define FRC971_INPUT_GYRO_BOARD_DATA_H_

#include "aos/common/byteorder.h"

namespace frc971 {

// The struct that the gyro board sends out with all of the data in it.
struct GyroBoardData {
	int64_t gyro_angle;

	int32_t left_drive;
	int32_t right_drive;
	int32_t shooter_angle;
	int32_t shooter;
	int32_t indexer;
	int32_t wrist;

	int32_t capture_top_rise;
	int32_t capture_top_fall;
	int32_t capture_bottom_fall_delay;
	int32_t capture_wrist_rise;
	int32_t capture_shooter_angle_rise;

	uint8_t top_rise_count;

	uint8_t top_fall_count;

	uint8_t bottom_rise_count;

	uint8_t bottom_fall_delay_count;
	uint8_t bottom_fall_count;

	uint8_t wrist_rise_count;

	uint8_t shooter_angle_rise_count;

  union {
    struct {
      uint8_t wrist_hall_effect : 1;
      uint8_t angle_adjust_bottom_hall_effect : 1;
      uint8_t top_disc : 1;
      uint8_t bottom_disc : 1;
      uint8_t loader_top : 1;
      uint8_t loader_bottom : 1;
    };
    uint32_t digitals;
  };

  void NetworkToHost() {
    // Apparently it sends the information out in little endian.
#if 0
    using ::aos::ntoh;

    gyro_angle = ntoh(gyro_angle);

    right_drive = ntoh(right_drive);
    left_drive = ntoh(left_drive);
    shooter_angle = ntoh(shooter_angle);
    shooter = ntoh(shooter);
    indexer = ntoh(indexer);
    wrist = ntoh(wrist);

    capture_top_rise = ntoh(capture_top_rise);
    capture_top_fall = ntoh(capture_top_fall);
    capture_bottom_fall_delay = ntoh(capture_bottom_fall_delay);
    capture_wrist_rise = ntoh(capture_wrist_rise);
    capture_shooter_angle_rise = ntoh(capture_shooter_angle_rise);

    digitals = ntoh(digitals);
#endif
  }
} __attribute__((__packed__));

}  // namespace frc971

#endif  // FRC971_INPUT_GYRO_BOARD_DATA_H_
