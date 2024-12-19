#ifndef FRC971_ORIN_APRILTAG_INPUT_FORMAT_H_
#define FRC971_ORIN_APRILTAG_INPUT_FORMAT_H_

namespace frc971::apriltag {

enum class InputFormat {
  Mono8,
  Mono16,
  YCbCr422,
  BGR8,
  BGRA8,
};

} // namespace frc971::apriltag

#endif