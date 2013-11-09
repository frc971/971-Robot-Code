#include <inttypes.h>

#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/wrapping_counter.h"

#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/shooter/shooter_motor.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/input/usb_receiver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::bot3::control_loops::drivetrain;
using ::bot3::control_loops::shooter;
using ::frc971::sensors::gyro;
using ::aos::util::WrappingCounter;
using ::frc971::USBReceiver;

namespace bot3 {
namespace {

const double kWheelRadius = 1.964;
const double kTapeThickness = 2.696;

inline double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      (32.0 / 44.0 /*encoder gears*/) * // the encoders are on the wheels.
      (3.5 /*wheel diameter*/ * 2.54 / 100 * M_PI);
}

inline double shooter_translate(int32_t in) {
  return 1.0 / (static_cast<double>(in) / (10 ^ 8)/*ticks per sec*/)
    / (1 - (kTapeThickness / (2 * kWheelRadius * M_PI))) * (2 * M_PI);  
}

inline double gyro_translate(int64_t in) {
  return in / 16.0 / 1000.0 / (180.0 / M_PI);
}

}  // namespace

class GyroSensorReceiver : public USBReceiver {
 public:
  GyroSensorReceiver() : USBReceiver(1) {}

  virtual void ProcessData(const ::aos::time::Time &/*timestamp*/) override {
    gyro.MakeWithBuilder()
        .angle(gyro_translate(data()->gyro_angle))
        .Send();

    LOG(DEBUG, "right drive: %f, left drive: %f\n", 
        drivetrain_translate(data()->main.shooter_angle),
        drivetrain_translate(data()->main.indexer));
    drivetrain.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(data()->main.wrist))
        .left_encoder(drivetrain_translate(data()->main.shooter))
        .Send();
    /*drivetrain.position.MakeWithBuilder()
        .right_encoder(0)
        .left_encoder(0)
        .Send();*/

    shooter.position.MakeWithBuilder()
        .velocity(shooter_translate(data()->bot3.shooter_cycle_ticks))
        .Send();
  }
};

}  // namespace bot3

int main() {
  ::aos::Init(frc971::USBReceiver::kRelativePriority);
  ::bot3::GyroSensorReceiver receiver;
  while (true) {
    receiver.RunIteration();
  }
  ::aos::Cleanup();
}
