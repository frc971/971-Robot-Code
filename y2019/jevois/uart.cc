#include "y2019/jevois/uart.h"

namespace frc971 {
namespace jevois {

UartBuffer UartPackToTeensy(const Frame & /*message*/) { return UartBuffer(); }

tl::optional<CameraCalibration> UartUnpackToCamera(
    const UartBuffer & /*message*/) {
  return tl::nullopt;
}

}  // namespace jevois
}  // namespace frc971
