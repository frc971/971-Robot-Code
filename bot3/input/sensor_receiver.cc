#include <inttypes.h>

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/common/time.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/controls/output_check.q.h"

#include "bbb/sensor_reader.h"

#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/drivetrain/drivetrain_constants.h"
#include "bot3/queues/to_log.q.h"
#include "bot3/shifter_hall_effect.h"
#include "frc971/queues/other_sensors.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::bot3::control_loops::drivetrain;
using ::frc971::sensors::gyro_reading;
using ::aos::util::WrappingCounter;

namespace bot3 {
namespace {

// TODO (danielp): Find out the real ratios here.
double drivetrain_translate(int32_t in) {
  return static_cast<double>(in)
      / (256.0 /*cpr*/ * 4.0 /*quad*/)
      * (18.0 / 50.0 /*output stage*/) * (56.0 / 30.0 /*encoder gears*/)
      // * constants::GetValues().drivetrain_encoder_ratio
      * (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

static const double kVcc = 5.15;

// Translates values from the ADC into voltage.
double adc_translate(uint16_t in) {
  if (false) {
    // This is the simple theoretical math.
    static const uint16_t kMaximumValue = 0x3FF;
    static const double kR1 = 5, kR2 = 6.65;
    const double raw =
        (kVcc * static_cast<double>(in) / static_cast<double>(kMaximumValue));
    return (raw * (kR1 + kR2) - (kVcc / 2) * kR2) / kR1;
  } else {
    // This is from a linear regression calculated with some actual data points.
    static const double kM = 0.012133, kB = -3.6813;
    return static_cast<double>(in) * kM + kB;
  }
}

double battery_translate(uint16_t in_high, uint16_t in_low) {
  const double high = adc_translate(in_high), low = adc_translate(in_low);
  static const double kDividerBig = 5.55, kDividerSmall = 2.66;
  return (high - low) * (kDividerBig + kDividerSmall) / kDividerSmall +
      kDividerBig / kDividerSmall * kVcc;
}

double gyro_translate(int64_t in) {
  return in / 16.0 / 1000.0 / (180.0 / M_PI);
}

double hall_translate(const constants::ShifterHallEffect & k,
    uint16_t in_value) {
    return (in_value - static_cast<double>(k.low)) /
        static_cast<double>(k.high - k.low);
}

void PacketReceived(const ::bbb::DataStruct *data,
                    const ::aos::time::Time &cape_timestamp) {
  ::aos::time::TimeFreezer time_freezer;

  ::frc971::logging_structs::CapeReading reading_to_log(
      cape_timestamp, static_cast<uint16_t>(sizeof(*data)),
      data->main.low_left_drive_hall, data->main.high_left_drive_hall,
      data->main.low_right_drive_hall, data->main.high_right_drive_hall);
  LOG_STRUCT(DEBUG, "cape reading", reading_to_log);
  bool bad_gyro;
  // TODO(brians): Switch to LogInterval for these things.
  if (data->uninitialized_gyro) {
    LOG(DEBUG, "uninitialized gyro\n");
    bad_gyro = true;
  } else if (data->zeroing_gyro) {
    LOG(DEBUG, "zeroing gyro\n");
    bad_gyro = true;
  } else if (data->bad_gyro) {
    LOG(ERROR, "bad gyro\n");
    bad_gyro = true;
  } else if (data->old_gyro_reading) {
    LOG(WARNING, "old/bad gyro reading\n");
    bad_gyro = true;
  } else {
    bad_gyro = false;
  }

  if (!bad_gyro) {
    gyro_reading.MakeWithBuilder()
        .angle(gyro_translate(data->gyro_angle))
        .Send();
  }

  if (data->analog_errors != 0) {
    LOG(WARNING, "%" PRIu8 " analog errors\n", data->analog_errors);
  }

  if (data->main.output_check_pulse_length != 0) {
    auto message = ::aos::controls::output_check_received.MakeMessage();
    // TODO(brians): Fix this math to match what the cRIO actually does.
    // It's close but not quite right.
    message->pulse_length =
        static_cast<double>(data->main.output_check_pulse_length) / 10000.0;
    if (message->pulse_length > 2.7) {
      LOG(WARNING, "insane PWM pulse length %fms\n", message->pulse_length);
    } else {
      message->pwm_value = (message->pulse_length - 0.5) / 2.0 * 255.0 + 0.5;
      LOG_STRUCT(DEBUG, "received", *message);
      message.Send();
    }
  }

  drivetrain.position.MakeWithBuilder()
      .right_encoder(drivetrain_translate(data->main.right_drive))
      .left_encoder(-drivetrain_translate(data->main.left_drive))
      .left_shifter_position(hall_translate(control_loops::kBot3LeftDriveShifter,
                                            data->main.low_left_drive_hall))
      .right_shifter_position(hall_translate(control_loops::kBot3RightDriveShifter,
                                             data->main.low_right_drive_hall))
      .battery_voltage(battery_translate(data->main.battery_voltage_high,
                                         data->main.battery_voltage_low))
      .Send();
}

}  // namespace
}  // namespace bot3

int main() {
  ::aos::Init(::bbb::SensorReader::kRelativePriority);
  ::bbb::SensorReader reader("comp");
  while (true) {
    ::bot3::PacketReceived(reader.ReadPacket(), reader.GetCapeTimestamp());
  }
  ::aos::Cleanup();
}
