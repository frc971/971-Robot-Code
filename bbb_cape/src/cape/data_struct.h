// This isn't really a header file. It's designed to be #included directly into
// other code (possibly in a namespace or whatever), so it doesn't have include
// guards.
// This means that it can not #include anything else because it (sometimes) gets
// #included inside a namespace.
// <stdint.h> must be #included by the containing file.
// In the cape code, bbb_cape/src/cape/fill_packet.h #includes this file.
// In the prime code, bbb_cape/src/bbb/data_struct.h #includes this file.

#pragma pack(push, 1)
typedef struct {
  uint8_t posedges, negedges;
} HallEffectEdges;
typedef struct {
  int32_t position, posedge_position, negedge_position;

  HallEffectEdges front, calibration, back;

  struct {
    uint16_t front : 1;
    uint16_t calibration : 1;
    uint16_t back : 1;
  } bools;
} SingleClawPosition;

// Be careful with declaration order in here. ARM doesn't like unaligned
// accesses and this structure is packed, so messing the order up will cause the
// compiler to generate very inefficient code to access fields.
struct DATA_STRUCT_NAME {
  int64_t gyro_angle;

  union {
    struct {
      // In 10us since the cape last reset.
      uint64_t timestamp;

      // The CRC32 (same algorithm as the checksum for the packet) of the whole
      // contents of flash for the main code (aka what's in the .hex file).
      uint32_t flash_checksum;

      struct {
        // If the current gyro_angle has been not updated because of a bad
        // reading from the sensor.
        uint8_t old_gyro_reading : 1;
        // If the gyro is still initializing.
        // If this is 1, then all of the other gyro data is invalid.
        uint8_t uninitialized_gyro : 1;
        // If the gyro is still zeroing.
        // If this is 1, then all of the other gyro data is invalid.
        uint8_t zeroing_gyro : 1;
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
    // This is for the test code that basically just sends all of the values
    // over to make sure that everything is working.
    struct {
      int32_t encoders[8];

      uint16_t analogs[8];

      uint32_t digitals;

      int32_t posedge_value, negedge_value;
      uint8_t posedge_count, negedge_count;
    } test;

    // This is for the comp and practice robots.
    struct {
      SingleClawPosition top_claw, bottom_claw;

      int32_t left_drive;
      int32_t right_drive;

      // The length of the pulse from the ultrasonic sensor in 100kHZ ticks.
      uint32_t ultrasonic_pulse_length;

      int32_t shooter_position, shooter_posedge_position,
          shooter_negedge_position;

      uint16_t left_drive_hall;
      uint16_t right_drive_hall;

      uint16_t battery_voltage;

      HallEffectEdges plunger, pusher_distal, pusher_proximal, latch;

      struct {
        uint8_t plunger : 1;
        uint8_t pusher_distal : 1;
        uint8_t pusher_proximal : 1;
        uint8_t latch : 1;
      } bools;
    } main;
  };
} __attribute__((aligned(8)));
#pragma pack(pop)

// The number of bytes that we actually send (so it stays consistent) (including
// the byte-stuffing overhead and the CRC on the end).
// This will always be a multiple of 4.
#define DATA_STRUCT_SEND_SIZE 148

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
