#ifndef __ANALOG_H__
#define __ANALOG_H__

extern int64_t gyro_angle;

struct DataStruct {
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

  int8_t top_rise_count;

  int8_t top_fall_count;

  int8_t bottom_rise_count;

  int8_t bottom_fall_delay_count;
  int8_t bottom_fall_count;

  int8_t wrist_rise_count;

  int8_t shooter_angle_rise_count;

  union {
    struct {
      uint8_t wrist_hall_effect : 1;
      uint8_t angle_adjust_bottom_hall_effect : 1;
      uint8_t top_disc : 1;
      uint8_t bottom_disc : 1;
    };
    uint32_t digitals;
  };
} __attribute__((__packed__));
// Gets called in the USB data output ISR. Assumes that it will not be preempted
// except by very high priority things.
void fillSensorPacket(struct DataStruct *packet);

void analog_init(void);
int analog(int channel);

int digital(int channel);

void encoder_init(void);
// For debugging only.
// Returns the current values of the inputs for the given encoder (as the low 2
// bits).
int encoder_bits(int channel);
// Returns the current position of the given encoder.
int32_t encoder_val(int channel);

int dip(int channel);
#endif  // __ANALOG_H__
