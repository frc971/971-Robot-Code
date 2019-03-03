#include "y2019/jevois/spi.h"

#include "aos/util/bitpacking.h"
#include "y2019/jevois/jevois_crc.h"
#ifdef __linux__
#include "aos/logging/logging.h"
#else
#define CHECK(...)
#define CHECK_GE(...)
#endif

// SPI transfer format (6x 8 bit frames):
// 1. 1-byte brightness for each beacon channel.
// 2. 1-byte specifying on/off for each light ring.
// 3. 2-byte CRC
//
// SPI transfer format (41x 8 bit frames):
// 1. Camera frame 0
// 2. Camera frame 1
// 3. Camera frame 2
// 4. 2-byte CRC-16
// Each camera frame (13x 8 bit frames):
//   1. Duration for how old the frame is. This is a value received from the
//      camera, added to the time between the first character being received
//      by the MCU to the CS line being asserted. Specifically it's an 8 bit
//      unsigned number of ms.
//   2. Target 0
//   3. Target 1
//   4. Target 2
//   Each target (4x 8 bit frames):
//     1. 10 bits heading
//     2. 8 bits distance
//     3. 6 bits skew
//     4. 6 bits height
//     5. 1 bit target valid (a present frame has all-valid targets)
//     6. 1 bit target present (a present frame can have from 0 to 3
//          targets, depending on how many were found)
//   Note that empty frames are still sent to indicate that the camera is
//   still working even though it doesn't see any targets.

namespace frc971 {
namespace jevois {
namespace {

constexpr float heading_min() { return -3; }
constexpr float heading_max() { return 3; }
constexpr int heading_bits() { return 10; }
constexpr int heading_offset() { return 0; }
void heading_pack(float heading, gsl::span<char> destination) {
  const auto integer = aos::FloatToIntLinear<heading_bits()>(
      heading_min(), heading_max(), heading);
  aos::PackBits<uint32_t, heading_bits(), heading_offset()>(integer,
                                                            destination);
}
float heading_unpack(gsl::span<const char> source) {
  const auto integer =
      aos::UnpackBits<uint32_t, heading_bits(), heading_offset()>(source);
  return aos::IntToFloatLinear<heading_bits()>(heading_min(), heading_max(),
                                               integer);
}

constexpr float distance_min() { return 0; }
constexpr float distance_max() {
  // The field is 18.4m diagonally.
  return 18.4;
}
constexpr int distance_bits() { return 8; }
constexpr int distance_offset() { return heading_offset() + heading_bits(); }
void distance_pack(float distance, gsl::span<char> destination) {
  const auto integer = aos::FloatToIntLinear<distance_bits()>(
      distance_min(), distance_max(), distance);
  aos::PackBits<uint32_t, distance_bits(), distance_offset()>(integer,
                                                              destination);
}
float distance_unpack(gsl::span<const char> source) {
  const auto integer =
      aos::UnpackBits<uint32_t, distance_bits(), distance_offset()>(source);
  return aos::IntToFloatLinear<distance_bits()>(distance_min(), distance_max(),
                                                integer);
}

constexpr float skew_min() { return -3; }
constexpr float skew_max() { return 3; }
constexpr int skew_bits() { return 6; }
constexpr int skew_offset() { return distance_offset() + distance_bits(); }
void skew_pack(float skew, gsl::span<char> destination) {
  const auto integer =
      aos::FloatToIntLinear<skew_bits()>(skew_min(), skew_max(), skew);
  aos::PackBits<uint32_t, skew_bits(), skew_offset()>(integer, destination);
}
float skew_unpack(gsl::span<const char> source) {
  const auto integer =
      aos::UnpackBits<uint32_t, skew_bits(), skew_offset()>(source);
  return aos::IntToFloatLinear<skew_bits()>(skew_min(), skew_max(), integer);
}

constexpr float height_min() { return 0; }
constexpr float height_max() { return 1.5; }
constexpr int height_bits() { return 6; }
constexpr int height_offset() { return skew_offset() + skew_bits(); }
void height_pack(float height, gsl::span<char> destination) {
  const auto integer =
      aos::FloatToIntLinear<height_bits()>(height_min(), height_max(), height);
  aos::PackBits<uint32_t, height_bits(), height_offset()>(integer, destination);
}
float height_unpack(gsl::span<const char> source) {
  const auto integer =
      aos::UnpackBits<uint32_t, height_bits(), height_offset()>(source);
  return aos::IntToFloatLinear<height_bits()>(height_min(), height_max(),
                                              integer);
}

constexpr int valid_bits() { return 1; }
constexpr int valid_offset() { return height_offset() + height_bits(); }
void valid_pack(bool valid, gsl::span<char> destination) {
  aos::PackBits<uint32_t, valid_bits(), valid_offset()>(valid, destination);
}
bool valid_unpack(gsl::span<const char> source) {
  return aos::UnpackBits<uint32_t, valid_bits(), valid_offset()>(source);
}

constexpr int present_bits() { return 1; }
constexpr int present_offset() { return valid_offset() + valid_bits(); }
void present_pack(bool present, gsl::span<char> destination) {
  aos::PackBits<uint32_t, present_bits(), present_offset()>(present,
                                                            destination);
}
bool present_unpack(gsl::span<const char> source) {
  return aos::UnpackBits<uint32_t, present_bits(), present_offset()>(source);
}

constexpr int next_offset() { return present_offset() + present_bits(); }
static_assert(next_offset() <= 32, "Target is too big");

}  // namespace

SpiTransfer SpiPackToRoborio(const TeensyToRoborio &message) {
  SpiTransfer transfer;
  gsl::span<char> remaining_space = transfer;
  for (int frame = 0; frame < 3; ++frame) {
    for (int target = 0; target < 3; ++target) {
      remaining_space[0] = 0;
      remaining_space[1] = 0;
      remaining_space[2] = 0;
      remaining_space[3] = 0;

      if (static_cast<int>(message.frames.size()) > frame) {
        valid_pack(true, remaining_space);
        if (static_cast<int>(message.frames[frame].targets.size()) > target) {
          heading_pack(message.frames[frame].targets[target].heading,
                       remaining_space);
          distance_pack(message.frames[frame].targets[target].distance,
                        remaining_space);
          skew_pack(message.frames[frame].targets[target].skew,
                    remaining_space);
          height_pack(message.frames[frame].targets[target].height,
                      remaining_space);
          present_pack(true, remaining_space);
        } else {
          present_pack(false, remaining_space);
        }
      } else {
        valid_pack(false, remaining_space);
      }

      remaining_space = remaining_space.subspan(4);
    }
    if (static_cast<int>(message.frames.size()) > frame) {
      const uint8_t age_count = message.frames[frame].age.count();
      memcpy(&remaining_space[0], &age_count, 1);
    } else {
      remaining_space[0] = 0;
    }
    remaining_space = remaining_space.subspan(1);
  }
  {
    uint16_t crc = jevois_crc_init();
    crc = jevois_crc_update(crc, transfer.data(),
                            transfer.size() - remaining_space.size());
    crc = jevois_crc_finalize(crc);
    CHECK_GE(static_cast<size_t>(remaining_space.size()), sizeof(crc));
    memcpy(&remaining_space[0], &crc, sizeof(crc));
    remaining_space = remaining_space.subspan(sizeof(crc));
  }
  CHECK(remaining_space.empty());
  return transfer;
}

tl::optional<TeensyToRoborio> SpiUnpackToRoborio(
    gsl::span<const char, spi_transfer_size()> transfer) {
  TeensyToRoborio message;
  gsl::span<const char> remaining_input = transfer;
  for (int frame = 0; frame < 3; ++frame) {
    const bool have_frame = valid_unpack(remaining_input);
    if (have_frame) {
      message.frames.push_back({});
    }
    for (int target = 0; target < 3; ++target) {
      if (present_unpack(remaining_input)) {
        if (have_frame) {
          message.frames.back().targets.push_back({});
          message.frames.back().targets.back().heading =
              heading_unpack(remaining_input);
          message.frames.back().targets.back().distance =
              distance_unpack(remaining_input);
          message.frames.back().targets.back().skew =
              skew_unpack(remaining_input);
          message.frames.back().targets.back().height =
              height_unpack(remaining_input);
        }
      }

      remaining_input = remaining_input.subspan(4);
    }
    if (have_frame) {
      uint8_t age_count;
      memcpy(&age_count, &remaining_input[0], 1);
      message.frames.back().age = camera_duration(age_count);
    }
    remaining_input = remaining_input.subspan(1);
  }
  {
    uint16_t calculated_crc = jevois_crc_init();
    calculated_crc =
        jevois_crc_update(calculated_crc, transfer.data(),
                          transfer.size() - remaining_input.size());
    calculated_crc = jevois_crc_finalize(calculated_crc);
    uint16_t received_crc;
    CHECK_GE(static_cast<size_t>(remaining_input.size()), sizeof(received_crc));
    memcpy(&received_crc, &remaining_input[0], sizeof(received_crc));
    remaining_input = remaining_input.subspan(sizeof(received_crc));
    CHECK(remaining_input.empty());
    if (calculated_crc != received_crc) {
      return tl::nullopt;
    }
  }
  return message;
}

SpiTransfer SpiPackToTeensy(const RoborioToTeensy &message) {
  SpiTransfer transfer;
  gsl::span<char> remaining_space = transfer;
  for (size_t i = 0; i < message.beacon_brightness.size(); ++i) {
    remaining_space[0] = message.beacon_brightness[i];
    remaining_space = remaining_space.subspan(1);
  }
  remaining_space[0] = message.light_rings.to_ulong() & 0xFF;
  remaining_space = remaining_space.subspan(1);
  {
    const int64_t realtime_now =
        message.realtime_now.time_since_epoch().count();
    memcpy(remaining_space.data(), &realtime_now, sizeof(realtime_now));
    remaining_space = remaining_space.subspan(sizeof(realtime_now));
  }
  memcpy(remaining_space.data(), &message.camera_command, 1);
  remaining_space = remaining_space.subspan(1);
  {
    uint16_t crc = jevois_crc_init();
    crc = jevois_crc_update(crc, transfer.data(),
                            transfer.size() - remaining_space.size());
    crc = jevois_crc_finalize(crc);
    CHECK_GE(static_cast<size_t>(remaining_space.size()), sizeof(crc));
    memcpy(&remaining_space[0], &crc, sizeof(crc));
    remaining_space = remaining_space.subspan(sizeof(crc));
  }
  return transfer;
}

tl::optional<RoborioToTeensy> SpiUnpackToTeensy(
    gsl::span<const char, spi_transfer_size()> transfer) {
  RoborioToTeensy message;
  gsl::span<const char> remaining_input = transfer;
  for (size_t i = 0; i < message.beacon_brightness.size(); ++i) {
    message.beacon_brightness[i] = remaining_input[0];
    remaining_input = remaining_input.subspan(1);
  }
  message.light_rings = remaining_input[0];
  remaining_input = remaining_input.subspan(1);
  {
    int64_t realtime_now;
    memcpy(&realtime_now, remaining_input.data(), sizeof(realtime_now));
    message.realtime_now = aos::realtime_clock::time_point(
        aos::realtime_clock::duration(realtime_now));
    remaining_input = remaining_input.subspan(sizeof(realtime_now));
  }
  memcpy(&message.camera_command, remaining_input.data(), 1);
  remaining_input = remaining_input.subspan(1);
  {
    uint16_t calculated_crc = jevois_crc_init();
    calculated_crc =
        jevois_crc_update(calculated_crc, transfer.data(),
                          transfer.size() - remaining_input.size());
    calculated_crc = jevois_crc_finalize(calculated_crc);
    uint16_t received_crc;
    CHECK_GE(static_cast<size_t>(remaining_input.size()), sizeof(received_crc));
    memcpy(&received_crc, &remaining_input[0], sizeof(received_crc));
    remaining_input = remaining_input.subspan(sizeof(received_crc));
    if (calculated_crc != received_crc) {
      return tl::nullopt;
    }
  }
  return message;
}

}  // namespace jevois
}  // namespace frc971
