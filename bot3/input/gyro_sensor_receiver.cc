#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/wrapping_counter.h"

#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/input/usb_receiver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::bot3::control_loops::drivetrain;
using ::frc971::sensors::gyro;
using ::aos::util::WrappingCounter;
using ::frc971::USBReceiver;

namespace bot3 {
namespace {

inline double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      (19.0 / 50.0) /*output reduction*/ * (64.0 / 24.0) /*encoder gears*/ *
      (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

inline double wrist_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      (14.0 / 50.0 * 20.0 / 84.0) /*gears*/ * (2 * M_PI);
}

inline double angle_adjust_translate(int32_t in) {
  static const double kCableDiameter = 0.060;
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      ((0.75 + kCableDiameter) / (16.61125 + kCableDiameter)) /*pulleys*/ *
      (2 * M_PI);
}

inline double shooter_translate(int32_t in) {
 return static_cast<double>(in) / (32.0 /*cpr*/ * 4.0 /*quad*/) *
      (15.0 / 34.0) /*gears*/ * (2 * M_PI);
}

inline double index_translate(int32_t in) {
  return -static_cast<double>(in) / (128.0 /*cpr*/ * 4.0 /*quad*/) *
      (1.0) /*gears*/ * (2 * M_PI);
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
        .right_encoder(drivetrain_translate(data()->main.right_drive))
        .left_encoder(-drivetrain_translate(data()->main.left_drive))
        .Send();
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
