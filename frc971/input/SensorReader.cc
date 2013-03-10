#define __STDC_LIMIT_MACROS

#include <arpa/inet.h>

#include "aos/aos_core.h"
#include "aos/common/inttypes.h"
#include "aos/common/input/SensorInput.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/sensor_values.h"

#define M_PI 3.14159265358979323846

using ::frc971::control_loops::drivetrain;

namespace frc971 {

namespace {
inline double drivetrain_translate(int32_t in) {
  // TODO(2013) fix the math
  return static_cast<double>(in) / (256.0 * 4.0 * 44.0 / 32.0) *
      (3.5 * 2.54 / 100.0 * M_PI);
}
} // namespace

class SensorReader : public aos::SensorInput<sensor_values> {
  virtual void RunIteration(sensor_values &sensors) {
    for (size_t i = 0; i < sizeof(sensors.encoders) / sizeof(sensors.encoders[0]); ++i) {
      sensors.encoders[i] = ntohl(sensors.encoders[i]);
    }

    // TODO(aschuh): Convert to meters.
    const double left_encoder = drivetrain_translate(sensors.lencoder);
    const double right_encoder = drivetrain_translate(sensors.rencoder);
    drivetrain.position.MakeWithBuilder()
        .left_encoder(left_encoder)
        .right_encoder(right_encoder)
        .Send();
  }
};

} // namespace frc971

AOS_RUN(frc971::SensorReader)
