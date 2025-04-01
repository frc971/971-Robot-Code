#ifndef FRC971_CTRE_CANRANGE_H_
#define FRC971_CTRE_CANRANGE_H_

#include "ctre/phoenix6/CANrange.hpp"

#include "aos/logging/logging.h"
#include "frc971/wpilib/can_range_static.h"

namespace frc971::wpilib {

// Minimum signal strength for a valid measurement.
constexpr double kMinSignalStrength = 2500;

// The sensor only updates at 100hz so we need to differentiate it from the
// regular can update freq.
constexpr units::frequency::hertz_t kCANRangeUpdateFreqHz = 100_Hz;

class CANRange {
 public:
  CANRange(int device_id, std::string_view canbus,
           std::vector<::ctre::phoenix6::BaseStatusSignal *> *signals);

  void PrintConfigs();
  void WriteConfigs();

  void SerializeStatus(wpilib::CANRangeStatusStatic *status);

  int device_id() const { return device_id_; }
  float supply_voltage() const { return supply_voltage_.GetValue().value(); }
  float distance() const { return distance_.GetValue().value(); }
  int64_t measurement_time() const {
    std::chrono::nanoseconds timestamp(
        static_cast<int64_t>(measurement_time_.GetValue().value() * 1e9));
    return timestamp.count();
  }
  float signal_strength() const { return signal_strength_.GetValue().value(); }
  bool is_detected() const { return is_detected_.GetValue(); }
  frc971::wpilib::MeasurementHealth measurement_health() const {
    return static_cast<frc971::wpilib::MeasurementHealth>(
        measurement_health_.GetValue().value);
  }
  float ambient_signal() const { return ambient_signal_.GetValue().value(); }
  float distance_std_dev() const {
    return distance_std_dev_.GetValue().value();
  }

  int64_t GetTimestamp() {
    std::chrono::nanoseconds latest_timestamp =
        distance_.GetTimestamp().GetTime();

    return latest_timestamp.count();
  }

 private:
  int device_id_;
  ::ctre::phoenix6::hardware::CANrange canrange_;

  ::ctre::phoenix6::StatusSignal<units::voltage::volt_t> supply_voltage_;
  ::ctre::phoenix6::StatusSignal<units::length::meter_t> distance_;
  ::ctre::phoenix6::StatusSignal<units::time::second_t> measurement_time_;
  ::ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t>
      signal_strength_;
  ::ctre::phoenix6::StatusSignal<bool> is_detected_;
  ::ctre::phoenix6::StatusSignal<
      ::ctre::phoenix6::signals::MeasurementHealthValue>
      measurement_health_;
  ::ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t>
      ambient_signal_;
  ::ctre::phoenix6::StatusSignal<units::length::meter_t> distance_std_dev_;
};

}  // namespace frc971::wpilib

#endif  // FRC971_CTRE_CANRANGE_H_
