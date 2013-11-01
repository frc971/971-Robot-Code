#include <inttypes.h>

#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/wrapping_counter.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/input/usb_receiver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::wrist;
using ::frc971::control_loops::angle_adjust;
using ::frc971::control_loops::shooter;
using ::frc971::control_loops::index_loop;
using ::frc971::sensors::gyro;
using ::aos::util::WrappingCounter;

namespace frc971 {
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

// Translates values from the ADC into voltage.
inline double adc_translate(uint16_t in) {
  static const double kVRefN = 0;
  static const double kVRefP = 3.3;
  static const int kMaximumValue = 0x3FF;
  return kVRefN +
      (static_cast<double>(in) / static_cast<double>(kMaximumValue) *
       (kVRefP - kVRefN));
}

inline double gyro_translate(int64_t in) {
  return in / 16.0 / 1000.0 / (180.0 / M_PI);
}

inline double battery_translate(uint16_t in) {
  return adc_translate(in) * 80.8 / 17.8;
}

}  // namespace

class GyroSensorReceiver : public USBReceiver {
 public:
  GyroSensorReceiver() : USBReceiver(2) {}

  virtual void PacketReceived(const ::aos::time::Time &/*timestamp*/) override {
    if (data()->bad_gyro) {
      LOG(ERROR, "bad gyro\n");
      bad_gyro_ = true;
      gyro.MakeWithBuilder().angle(0).Send();
    } else if (data()->old_gyro_reading) {
      LOG(WARNING, "old gyro reading\n");
      bad_gyro_ = true;
    } else {
      bad_gyro_ = false;
    }
  }

  virtual void ProcessData(const ::aos::time::Time &/*timestamp*/) override {
    if (!bad_gyro_) {
      gyro.MakeWithBuilder()
          .angle(gyro_translate(data()->gyro_angle))
          .Send();
    }

    drivetrain.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(data()->main.right_drive))
        .left_encoder(-drivetrain_translate(data()->main.left_drive))
        .Send();

    wrist.position.MakeWithBuilder()
        .pos(wrist_translate(data()->main.wrist))
        .hall_effect(data()->main.wrist_hall_effect)
        .calibration(wrist_translate(data()->main.capture_wrist_rise))
        .Send();

    angle_adjust.position.MakeWithBuilder()
        .angle(angle_adjust_translate(data()->main.shooter_angle))
        .bottom_hall_effect(data()->main.angle_adjust_bottom_hall_effect)
        .middle_hall_effect(false)
        .bottom_calibration(angle_adjust_translate(
                data()->main.capture_shooter_angle_rise))
        .middle_calibration(angle_adjust_translate(
                0))
        .Send();

    shooter.position.MakeWithBuilder()
        .position(shooter_translate(data()->main.shooter))
        .Send();

    index_loop.position.MakeWithBuilder()
        .index_position(index_translate(data()->main.indexer))
        .top_disc_detect(data()->main.top_disc)
        .top_disc_posedge_count(top_rise_.Update(data()->main.top_rise_count))
        .top_disc_posedge_position(
            index_translate(data()->main.capture_top_rise))
        .top_disc_negedge_count(top_fall_.Update(data()->main.top_fall_count))
        .top_disc_negedge_position(
            index_translate(data()->main.capture_top_fall))
        .bottom_disc_detect(data()->main.bottom_disc)
        .bottom_disc_posedge_count(
            bottom_rise_.Update(data()->main.bottom_rise_count))
        .bottom_disc_negedge_count(
            bottom_fall_.Update(data()->main.bottom_fall_count))
        .bottom_disc_negedge_wait_position(index_translate(
                data()->main.capture_bottom_fall_delay))
        .bottom_disc_negedge_wait_count(
            bottom_fall_delay_.Update(data()->main.bottom_fall_delay_count))
        .loader_top(data()->main.loader_top)
        .loader_bottom(data()->main.loader_bottom)
        .Send();

    LOG(DEBUG, "battery %f %f %" PRIu16 "\n",
        battery_translate(data()->main.battery_voltage),
        adc_translate(data()->main.battery_voltage),
        data()->main.battery_voltage);
    LOG(DEBUG, "halls l=%f r=%f\n",
        adc_translate(data()->main.left_drive_hall),
        adc_translate(data()->main.right_drive_hall));
  }

  bool bad_gyro_;

  WrappingCounter top_rise_;
  WrappingCounter top_fall_;
  WrappingCounter bottom_rise_;
  WrappingCounter bottom_fall_delay_;
  WrappingCounter bottom_fall_;
};

}  // namespace frc971

int main() {
  ::aos::Init(frc971::USBReceiver::kRelativePriority);
  ::frc971::GyroSensorReceiver receiver;
  while (true) {
    receiver.RunIteration();
  }
  ::aos::Cleanup();
}
