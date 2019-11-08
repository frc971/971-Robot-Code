#include "y2019/jevois/uart.h"

#include <array>

#include "aos/util/bitpacking.h"
#include "third_party/GSL/include/gsl/gsl"
#include "y2019/jevois/jevois_crc.h"
#ifdef __linux__
#include "aos/logging/logging.h"
#else
#define AOS_CHECK(...)
#define AOS_CHECK_GE(...)
#endif

namespace frc971 {
namespace jevois {

UartToTeensyBuffer UartPackToTeensy(const CameraFrame &message) {
  std::array<char, uart_to_teensy_size()> buffer;
  gsl::span<char> remaining_space = buffer;
  remaining_space[0] = message.targets.size();
  remaining_space = remaining_space.subspan(1);
  for (size_t i = 0; i < 3; ++i) {
    if (i < message.targets.size()) {
      memcpy(remaining_space.data(), &message.targets[i].distance,
             sizeof(float));
      remaining_space = remaining_space.subspan(sizeof(float));
      memcpy(remaining_space.data(), &message.targets[i].height, sizeof(float));
      remaining_space = remaining_space.subspan(sizeof(float));
      memcpy(remaining_space.data(), &message.targets[i].heading,
             sizeof(float));
      remaining_space = remaining_space.subspan(sizeof(float));
      memcpy(remaining_space.data(), &message.targets[i].skew, sizeof(float));
      remaining_space = remaining_space.subspan(sizeof(float));
    } else {
      remaining_space = remaining_space.subspan(sizeof(float) * 4);
    }
  }
  remaining_space[0] = message.age.count();
  remaining_space = remaining_space.subspan(1);
  {
    uint16_t crc = jevois_crc_init();
    crc = jevois_crc_update(crc, buffer.data(),
                            buffer.size() - remaining_space.size());
    crc = jevois_crc_finalize(crc);
    AOS_CHECK_GE(static_cast<size_t>(remaining_space.size()), sizeof(crc));
    memcpy(&remaining_space[0], &crc, sizeof(crc));
    remaining_space = remaining_space.subspan(sizeof(crc));
  }
  AOS_CHECK(remaining_space.empty());
  UartToTeensyBuffer result;
  result.set_size(
      CobsEncode<uart_to_teensy_size()>(buffer, result.mutable_backing_array())
          .size());
  return result;
}

std::optional<CameraFrame> UartUnpackToTeensy(
    gsl::span<const char> encoded_buffer) {
  std::array<char, uart_to_teensy_size()> buffer;
  if (static_cast<size_t>(
          CobsDecode<uart_to_teensy_size()>(encoded_buffer, &buffer).size()) !=
      buffer.size()) {
    return std::nullopt;
  }

  CameraFrame message;
  gsl::span<const char> remaining_input = buffer;
  const int number_targets = remaining_input[0];
  remaining_input = remaining_input.subspan(1);
  for (int i = 0; i < 3; ++i) {
    if (i < number_targets) {
      message.targets.push_back({});
      Target *const target = &message.targets.back();
      memcpy(&target->distance, remaining_input.data(), sizeof(float));
      remaining_input = remaining_input.subspan(sizeof(float));
      memcpy(&target->height, remaining_input.data(), sizeof(float));
      remaining_input = remaining_input.subspan(sizeof(float));
      memcpy(&target->heading, remaining_input.data(), sizeof(float));
      remaining_input = remaining_input.subspan(sizeof(float));
      memcpy(&target->skew, remaining_input.data(), sizeof(float));
      remaining_input = remaining_input.subspan(sizeof(float));
    } else {
      remaining_input = remaining_input.subspan(sizeof(float) * 4);
    }
  }
  message.age = camera_duration(remaining_input[0]);
  remaining_input = remaining_input.subspan(1);
  {
    uint16_t calculated_crc = jevois_crc_init();
    calculated_crc = jevois_crc_update(calculated_crc, buffer.data(),
                                       buffer.size() - remaining_input.size());
    calculated_crc = jevois_crc_finalize(calculated_crc);
    uint16_t received_crc;
    AOS_CHECK_GE(static_cast<size_t>(remaining_input.size()),
                 sizeof(received_crc));
    memcpy(&received_crc, &remaining_input[0], sizeof(received_crc));
    remaining_input = remaining_input.subspan(sizeof(received_crc));
    AOS_CHECK(remaining_input.empty());
    if (calculated_crc != received_crc) {
      return std::nullopt;
    }
  }
  return message;
}

UartToCameraBuffer UartPackToCamera(const CameraCalibration &message) {
  std::array<char, uart_to_camera_size()> buffer;
  gsl::span<char> remaining_space = buffer;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      memcpy(remaining_space.data(), &message.calibration(i, j), sizeof(float));
      remaining_space = remaining_space.subspan(sizeof(float));
    }
  }
  {
    const int64_t teensy_now = message.teensy_now.time_since_epoch().count();
    memcpy(remaining_space.data(), &teensy_now, sizeof(teensy_now));
    remaining_space = remaining_space.subspan(sizeof(teensy_now));
  }
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
    crc = jevois_crc_update(crc, buffer.data(),
                            buffer.size() - remaining_space.size());
    crc = jevois_crc_finalize(crc);
    AOS_CHECK_GE(static_cast<size_t>(remaining_space.size()), sizeof(crc));
    memcpy(&remaining_space[0], &crc, sizeof(crc));
    remaining_space = remaining_space.subspan(sizeof(crc));
  }
  AOS_CHECK(remaining_space.empty());
  UartToCameraBuffer result;
  result.set_size(
      CobsEncode<uart_to_camera_size()>(buffer, result.mutable_backing_array())
          .size());
  return result;
}

std::optional<CameraCalibration> UartUnpackToCamera(
    gsl::span<const char> encoded_buffer) {
  std::array<char, uart_to_camera_size()> buffer;
  if (static_cast<size_t>(
          CobsDecode<uart_to_camera_size()>(encoded_buffer, &buffer).size()) !=
      buffer.size()) {
    return std::nullopt;
  }

  CameraCalibration message;
  gsl::span<const char> remaining_input = buffer;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      memcpy(&message.calibration(i, j), remaining_input.data(), sizeof(float));
      remaining_input = remaining_input.subspan(sizeof(float));
    }
  }
  {
    int64_t teensy_now;
    memcpy(&teensy_now, remaining_input.data(), sizeof(teensy_now));
    message.teensy_now = aos::monotonic_clock::time_point(
        aos::monotonic_clock::duration(teensy_now));
    remaining_input = remaining_input.subspan(sizeof(teensy_now));
  }
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
    calculated_crc = jevois_crc_update(calculated_crc, buffer.data(),
                                       buffer.size() - remaining_input.size());
    calculated_crc = jevois_crc_finalize(calculated_crc);
    uint16_t received_crc;
    AOS_CHECK_GE(static_cast<size_t>(remaining_input.size()),
                 sizeof(received_crc));
    memcpy(&received_crc, &remaining_input[0], sizeof(received_crc));
    remaining_input = remaining_input.subspan(sizeof(received_crc));
    AOS_CHECK(remaining_input.empty());
    if (calculated_crc != received_crc) {
      return std::nullopt;
    }
  }
  return message;
}

}  // namespace jevois
}  // namespace frc971
