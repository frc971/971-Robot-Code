#include "frc971/input/sensor_unpacker.h"

#include <arpa/inet.h>

#include "aos/common/inttypes.h"

#include "frc971/control_loops/DriveTrain.q.h"

#define M_PI 3.14159265358979323846

using ::frc971::control_loops::drivetrain;

namespace frc971 {
namespace {

inline double drivetrain_translate(int32_t in) {
  // TODO(2013) fix the math
  return static_cast<double>(in) / (256.0 * 4.0 * 44.0 / 32.0) *
      (3.5 * 2.54 / 100.0 * M_PI);
}

}  // namespace

SensorUnpacker::SensorUnpacker() {}

void SensorUnpacker::UnpackFrom(sensor_values *values) {
  for (size_t i = 0; i < sizeof(values->encoders) / sizeof(values->encoders[0]); ++i) {
    values->encoders[i] = ntohl(values->encoders[i]);
  }

  // TODO(aschuh): Convert to meters.
  const double left_encoder = drivetrain_translate(values->lencoder);
  const double right_encoder = drivetrain_translate(values->rencoder);
  drivetrain.position.MakeWithBuilder()
      .left_encoder(left_encoder)
      .right_encoder(right_encoder)
      .Send();
}

}  // namespace frc971
