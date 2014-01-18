#include <inttypes.h>

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/common/time.h"

#include "bbb/sensor_reader.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro_angle.q.h"
#include "frc971/constants.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain;
using ::frc971::sensors::gyro;
using ::aos::util::WrappingCounter;

namespace frc971 {
namespace {

double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      constants::GetValues().drivetrain_encoder_ratio *
      (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

// Translates values from the ADC into voltage.
double adc_translate(uint16_t in) {
  static const double kVRefN = 0;
  static const double kVRefP = 3.3;
  static const int kMaximumValue = 0xFFF;
  static const double kDividerGnd = 31.6, kDividerOther = 28;
  return (kVRefN +
      (static_cast<double>(in) / static_cast<double>(kMaximumValue) *
       (kVRefP - kVRefN))) * (kDividerGnd + kDividerOther) / kDividerGnd;
}

double gyro_translate(int64_t in) {
  return in / 16.0 / 1000.0 / (180.0 / M_PI);
}

double battery_translate(uint16_t in) {
  static const double kDividerBig = 98.9, kDividerSmall = 17.8;
  return adc_translate(in) * kDividerBig / kDividerSmall;
}

double hall_translate(const constants::ShifterHallEffect &k, uint16_t in) {
  const double voltage = adc_translate(in);
  return (voltage - k.low) / (k.high - k.low);
}

void PacketReceived(const ::bbb::DataStruct *data,
                    const ::aos::time::Time &cape_timestamp) {
  LOG(DEBUG, "cape timestamp %010" PRId32 ".%09" PRId32 "s\n",
      cape_timestamp.sec(), cape_timestamp.nsec());
  bool bad_gyro;
  if (data->uninitialized_gyro) {
    LOG(DEBUG, "uninitialized gyro\n");
    bad_gyro = true;
  } else if (data->zeroing_gyro) {
    LOG(DEBUG, "zeroing gyro\n");
    bad_gyro = true;
  } else if (data->bad_gyro) {
    LOG(ERROR, "bad gyro\n");
    bad_gyro = true;
    gyro.MakeWithBuilder().angle(0).Send();
  } else if (data->old_gyro_reading) {
    LOG(WARNING, "old/bad gyro reading\n");
    bad_gyro = true;
  } else {
    bad_gyro = false;
  }

  if (!bad_gyro) {
    gyro.MakeWithBuilder()
        .angle(gyro_translate(data->gyro_angle))
        .Send();
  }

  drivetrain.position.MakeWithBuilder()
      .right_encoder(drivetrain_translate(data->main.right_drive))
      .left_encoder(-drivetrain_translate(data->main.left_drive))
      .left_shifter_position(hall_translate(constants::GetValues().left_drive,
                                            data->main.left_drive_hall))
      .right_shifter_position(hall_translate(
              constants::GetValues().right_drive, data->main.right_drive_hall))
      .battery_voltage(battery_translate(data->main.battery_voltage))
      .Send();
}

}  // namespace
}  // namespace frc971

int main() {
  ::aos::Init(::bbb::SensorReader::kRelativePriority);
  ::bbb::SensorReader reader("main");
  while (true) {
    ::frc971::PacketReceived(reader.ReadPacket(), reader.GetCapeTimestamp());
  }
  ::aos::Cleanup();
}
