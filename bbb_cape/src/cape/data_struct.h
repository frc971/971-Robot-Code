// This isn't really a header file. It's designed to be #included directly into
// other code (possibly in a namespace or whatever), so it doesn't have include
// guards.
// This means that it can not #include anything else because it (sometimes) gets
// #included inside a namespace.
// <stdint.h> must be #included by the containing file.
// In the cape code, fill_packet.h #includes this file.
// In the fitpc code, frc971/input/gyro_board_data.h #includes this file.

#pragma pack(push, 1)
// Be careful with declaration order in here. ARM doesn't like unaligned
// accesses!
struct DATA_STRUCT_NAME {
  int64_t gyro_angle;

  union {
    struct {
      // In us since the cape last reset.
      uint64_t timestamp;

      struct {
        // If the current gyro_angle has been not updated because of a bad
        // reading from the sensor.
        uint8_t old_gyro_reading : 1;
        // If we're not going to get any more good gyro_angles.
        uint8_t bad_gyro : 1;
      };
    };
    struct {
      uint64_t header1, header2;
    };
  };

  // We are 64-bit aligned at this point.

  union {
    struct {
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

      uint16_t battery_voltage;
      uint16_t left_drive_hall;
      uint16_t right_drive_hall;

      int8_t top_rise_count;

      int8_t top_fall_count;

      int8_t bottom_rise_count;

      int8_t bottom_fall_delay_count;
      int8_t bottom_fall_count;

      int8_t wrist_rise_count;

      int8_t shooter_angle_rise_count;

      struct {
        uint8_t wrist_hall_effect : 1;
        uint8_t angle_adjust_bottom_hall_effect : 1;
        uint8_t top_disc : 1;
        uint8_t bottom_disc : 1;
        uint8_t loader_top : 1;
        uint8_t loader_bottom : 1;
      };
    } main;
    
    struct {
      union {
        struct {
        };
        uint16_t booleans;
      };
    } bot3;
  };
} __attribute__((aligned(8)));
#pragma pack(pop)

// The number of bytes that we actually send (so it stays consistent) (including
// the byte-stuffing overhead and the CRC on the end).
#define DATA_STRUCT_SEND_SIZE 200

#ifdef __cplusplus
#define STATIC_ASSERT(cond, msg) static_assert(cond, #msg)
#endif
// 4 bytes of 0s at the beginning, 4 bytes of byte-stuffing overhead, and 4
// bytes of CRC on the end.
STATIC_ASSERT(
    (sizeof(struct DATA_STRUCT_NAME) + 8 + 4) <= DATA_STRUCT_SEND_SIZE,
    The_sensor_data_structure_is_too_big);
// The byte-stuffing and CRC both work in chunks of 4 bytes, so it has to be a
// multiple of that in size.
STATIC_ASSERT((sizeof(struct DATA_STRUCT_NAME) % 4) == 0,
              The_sensor_data_structure_is_not_a_multiple_of_4_bytes);
#ifdef __cplusplus
#undef STATIC_ASSERT
#endif
