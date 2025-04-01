#include "frc971/wpilib/can_range.h"

using frc971::wpilib::CANRange;
using frc971::wpilib::kCANRangeUpdateFreqHz;

CANRange::CANRange(int device_id, std::string_view canbus,
                   std::vector<::ctre::phoenix6::BaseStatusSignal *> *signals)
    : device_id_(device_id),
      canrange_(device_id, canbus),
      supply_voltage_(canrange_.GetSupplyVoltage()),
      distance_(canrange_.GetDistance()),
      measurement_time_(canrange_.GetMeasurementTime()),
      signal_strength_(canrange_.GetSignalStrength()),
      is_detected_(canrange_.GetIsDetected()),
      measurement_health_(canrange_.GetMeasurementHealth()),
      ambient_signal_(canrange_.GetAmbientSignal()),
      distance_std_dev_(canrange_.GetDistanceStdDev()) {
  CHECK(signals != nullptr);

  supply_voltage_.SetUpdateFrequency(kCANRangeUpdateFreqHz);
  signals->push_back(&supply_voltage_);

  distance_.SetUpdateFrequency(kCANRangeUpdateFreqHz);
  signals->push_back(&distance_);

  measurement_time_.SetUpdateFrequency(kCANRangeUpdateFreqHz);
  signals->push_back(&measurement_time_);

  signal_strength_.SetUpdateFrequency(kCANRangeUpdateFreqHz);
  signals->push_back(&signal_strength_);

  is_detected_.SetUpdateFrequency(kCANRangeUpdateFreqHz);
  signals->push_back(&is_detected_);

  measurement_health_.SetUpdateFrequency(kCANRangeUpdateFreqHz);
  signals->push_back(&measurement_health_);

  ambient_signal_.SetUpdateFrequency(kCANRangeUpdateFreqHz);
  signals->push_back(&ambient_signal_);
}

void CANRange::WriteConfigs() {
  ::ctre::phoenix6::configs::ProximityParamsConfigs proximity_params;
  proximity_params.MinSignalStrengthForValidMeasurement =
      frc971::wpilib::kMinSignalStrength;

  ::ctre::phoenix6::configs::ToFParamsConfigs tof_params;
  tof_params.UpdateMode =
      ::ctre::phoenix6::signals::UpdateModeValue::ShortRange100Hz;
  tof_params.UpdateFrequency = kCANRangeUpdateFreqHz;

  ::ctre::phoenix6::configs::CANrangeConfiguration configuration;
  configuration.ProximityParams = proximity_params;
  configuration.ToFParams = tof_params;

  ::ctre::phoenix::StatusCode status =
      canrange_.GetConfigurator().Apply(configuration);

  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to set talonfx motor configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }

  PrintConfigs();
}

void CANRange::PrintConfigs() {
  ::ctre::phoenix6::configs::CANrangeConfiguration configuration;
  ::ctre::phoenix::StatusCode status =
      canrange_.GetConfigurator().Refresh(configuration);

  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to get talonfx motor configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }
  AOS_LOG(INFO, "configuration: %s", configuration.ToString().c_str());
}

void CANRange::SerializeStatus(frc971::wpilib::CANRangeStatusStatic *status) {
  status->set_id(device_id_);
  status->set_supply_voltage(supply_voltage());
  status->set_distance(distance());
  status->set_signal_strength(signal_strength());
  status->set_is_detected(is_detected());
  status->set_measurement_health(measurement_health());
  status->set_ambient_signal(ambient_signal());
  status->set_distance_std_dev(distance_std_dev());
  status->set_measurement_timestamp(measurement_time());
  status->set_timestamp(GetTimestamp());
}
