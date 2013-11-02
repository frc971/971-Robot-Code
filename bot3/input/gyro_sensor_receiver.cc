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

//TODO (danielp): Figure out whether the bigger gear is on the 
// encoder or not.
inline double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      (32.0 / 44.0 /*encoder gears*/) * // the encoders are on the wheels.
      (3.5 /*wheel diameter*/ * 2.54 / 100 * M_PI);
}

// TODO (danielp): This needs to be modified eventually.
inline double shooter_translate(int32_t in) {
 return static_cast<double>(in) / (32.0 /*cpr*/ * 4.0 /*quad*/) *
      (15.0 / 34.0) /*gears*/ * (2 * M_PI);
}

}  // namespace

class GyroSensorReceiver : public USBReceiver {
  virtual void ProcessData() override {
    if (data()->robot_id != 0) {
      LOG(ERROR, "gyro board sent data for robot id %hhd!"
          " dip switches are %x\n",
          data()->robot_id, data()->base_status & 0xF);
      return;
    } else {
      LOG(DEBUG, "processing a packet dip switches %x\n",
          data()->base_status & 0xF);
    }

    gyro.MakeWithBuilder()
        .angle(data()->gyro_angle / 16.0 / 1000.0 / 180.0 * M_PI)
        .Send();

    drivetrain.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(data()->main.wrist))
        .left_encoder(drivetrain_translate(data()->main.shooter))
        .Send();
    LOG(DEBUG, "right: %lf left: %lf angle: %lld \n",
        drivetrain_translate(data()->main.wrist),
        drivetrain_translate(data()->main.shooter), data()->gyro_angle);

    shooter.position.MakeWithBuilder().position(drivetrain_translate(data()->main.shooter)).Send();
  }

  WrappingCounter top_rise_;
  WrappingCounter top_fall_;
  WrappingCounter bottom_rise_;
  WrappingCounter bottom_fall_delay_;
  WrappingCounter bottom_fall_; 
};

}  // namespace bot3

int main() {
  ::aos::Init();
  ::bot3::GyroSensorReceiver receiver;
  while (true) {
    receiver.RunIteration();
  }
  ::aos::Cleanup();
}
