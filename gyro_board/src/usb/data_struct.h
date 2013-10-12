// This isn't really a header file. It's designed to be #included directly into
// other code (possibly in a namespace or whatever), so it doesn't have include
// guards.
// This means that it can not #include anything else because it (sometimes) gets
// #included inside a namespace.
// In the gyro board code, fill_packet.h #includes this file.
// In the fitpc code, frc971/input/gyro_board_data.h #includes this file.

#pragma pack(push, 1)
// Be careful with declaration order in here. ARM doesn't like unaligned
// accesses!
struct DATA_STRUCT_NAME {
  int64_t gyro_angle;

  union {
    struct {
      // Which robot (+version) the gyro board is sending out data for.
      // We should keep this in the same place for all gyro board software
      // versions so that the fitpc can detect when it's reading from a gyro
      // board set up for a different robot than it is.
      // 0 = 2013 competition/practice robot
      // 1 = 2013 3rd robot
      uint8_t robot_id;
      // This information should also be kept in the same place from year to
      // year so that the fitpc code can record the dip switch values when it
      // detects the wrong robot id to make debugging easier.
      union {
        struct {
          uint8_t dip_switch0 : 1;
          uint8_t dip_switch1 : 1;
          uint8_t dip_switch2 : 1;
          uint8_t dip_switch3 : 1;
          // If the current gyro_angle has been not updated because of a bad
          // reading from the sensor.
          uint8_t old_gyro_reading : 1;
          // If we're not going to get any more good gyro_angles.
          uint8_t bad_gyro : 1;
        };
        uint8_t base_status;
      };
    };
    uint16_t header;
  };

  // This is a counter that gets incremented with each packet sent (and wraps
  // around when it reaches 255).
  uint8_t sequence;

  union {
    struct {
      union {
        struct {
          uint8_t wrist_hall_effect : 1;
          uint8_t angle_adjust_bottom_hall_effect : 1;
          uint8_t top_disc : 1;
          uint8_t bottom_disc : 1;
          uint8_t loader_top : 1;
          uint8_t loader_bottom : 1;
        };
        uint16_t booleans;
      };
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
    } main;
    
    struct {
      union {
        struct {
        };
        uint16_t booleans;
      };
    } bot3;
  };
};
#pragma pack(pop)

#ifdef __cplusplus
// TODO(brians): Consider using C1X's _Static_assert once we have a compiler
// (GCC 4.6) + flags that support it.
static_assert(sizeof(DATA_STRUCT_NAME) <= 64,
              "We only have room for 64 bytes in the USB packet.");
#endif  // defined(__cplusplus)
