#ifndef Y2019_JEVOIS_UART_H_
#define Y2019_JEVOIS_UART_H_

#include "aos/containers/sized_array.h"
#include "third_party/GSL/include/gsl/gsl"
#include <optional>
#include "y2019/jevois/cobs.h"
#include "y2019/jevois/structures.h"

// This file manages serializing and deserializing the various structures for
// transport via UART.

namespace frc971 {
namespace jevois {

constexpr size_t uart_to_teensy_size() {
  return 1 /* number of targets */ +
         3 /* targets */ * (sizeof(float) * 4 /* fields */) + 1 /* age */ +
         2 /* CRC-16 */;
}
using UartToTeensyBuffer =
    aos::SizedArray<char, CobsMaxEncodedSize(uart_to_teensy_size())>;

constexpr size_t uart_to_camera_size() {
  return sizeof(float) * 3 * 4 /* calibration */ +
         sizeof(int64_t) /* teensy_now */ + sizeof(int64_t) /* realtime_now */ +
         1 /* camera_command */ + 2 /* CRC-16 */;
}
using UartToCameraBuffer =
    aos::SizedArray<char, CobsMaxEncodedSize(uart_to_camera_size())>;

UartToTeensyBuffer UartPackToTeensy(const CameraFrame &message);
std::optional<CameraFrame> UartUnpackToTeensy(gsl::span<const char> buffer);

UartToCameraBuffer UartPackToCamera(const CameraCalibration &message);
std::optional<CameraCalibration> UartUnpackToCamera(
    gsl::span<const char> buffer);

}  // namespace jevois
}  // namespace frc971

#endif  // Y2019_JEVOIS_UART_H_
