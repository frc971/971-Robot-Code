// This isn't really a header file. It's designed to be #included directly into
// other code (possibly in a namespace or whatever), so it doesn't have include
// guards.
// This means that it can not #include anything else because it (sometimes) gets
// #included inside a namespace.
// <stdint.h> must be #included by the containing file.
// In the gyro board code, fill_packet.h #includes this file.
// In the fitpc code, frc971/input/gyro_board_data.h #includes this file.

#pragma pack(push, 1)
// Be careful with declaration order in here. ARM doesn't like unaligned
// accesses!
struct DATA_STRUCT_NAME {
  int64_t gyro_angle;

  union {
    struct {
      // This is the USB frame number for this data. It (theoretically) gets
      // incremented on every packet sent, but the gyro board will deal with it
      // correctly if it misses a frame or whatever by tracking the frame
      // numbers sent out by the host.
      // Negative numbers mean that the gyro board has no idea what the right
      // answer is.
      // This value going down at all indicates that the code on the gyro board
      // dealing with it reset.
      //
      // The USB 2.0 standard says that timing of frames is 1.000ms +- 500ns.
      // Testing with a fitpc and gyro board on 2013-10-30 by Brian gave 10us
      // (the resolution of the timer on the gyro board that was used) of drift
      // every 90-130 frames (~100ns per frame) and no jitter (and the timer on
      // the gyro board isn't necessarily that good). This is plenty accurate
      // for what we need for timing, so this number is what the code uses to do
      // all timing calculations.
      int32_t frame_number;

      // Checksum of this file calculated with sum(1).
      // The gyro board sets this and then the fitpc checks it to make sure that
      // they're both using the same version of this file.
      uint16_t checksum;

      // Which robot (+version) the gyro board is sending out data for.
      // We should keep this in the same place for all gyro board software
      // versions so that the fitpc can detect when it's reading from a gyro
      // board set up for a different robot (or version) than it is.
      // The numbers listed below each robot are the values that have been used
      // for it.
      //
      // 2013 competition/practice robot
      //   0
      //   2 added battery measurement + drivetrain analog hall effects
      // 2013 3rd robot
      //   1
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
        };
        uint8_t dip_switches;
      };
      struct {
        // If the current gyro_angle has been not updated because of a bad
        // reading from the sensor.
        uint8_t old_gyro_reading : 1;
        // If we're not going to get any more good gyro_angles.
        uint8_t bad_gyro : 1;

        // We're not sure what frame number this packet was sent in.
        uint8_t unknown_frame : 1;
      };
    };
    struct {
      uint64_t header0, header1;
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
};
#pragma pack(pop)

// This is how big the isochronous packets that we're going to send are.
// This number is more painful to change than the actual size of the struct
// because the code on both ends has to agree on this (or at least that's what
// Brian found empirically 2013-10-24).
#define DATA_STRUCT_SEND_SIZE 128

#ifdef __cplusplus
// TODO(brians): Consider using C1X's _Static_assert once we have a compiler
// (GCC 4.6) + flags that support it.
static_assert(sizeof(DATA_STRUCT_NAME) <= DATA_STRUCT_SEND_SIZE,
              "The sensor data structure is too big.");
#endif  // defined(__cplusplus)
