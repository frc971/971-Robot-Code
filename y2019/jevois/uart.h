#ifndef Y2019_JEVOIS_UART_H_
#define Y2019_JEVOIS_UART_H_

#include "aos/containers/sized_array.h"
#include "third_party/optional/tl/optional.hpp"
#include "y2019/jevois/structures.h"

// This file manages serializing and deserializing the various structures for
// transport via UART.

namespace frc971 {
namespace jevois {

constexpr size_t uart_max_size() {
  // TODO(Brian): Make this real.
  return 10;
}
using UartBuffer = aos::SizedArray<char, uart_max_size()>;

UartBuffer UartPackToTeensy(const Frame &message);
tl::optional<CameraCalibration> UartUnpackToCamera(const UartBuffer &message);

}  // namespace jevois
}  // namespace frc971

#endif  // Y2019_JEVOIS_UART_H_
