#include "frc971/wpilib/falcon.h"

using frc971::wpilib::Falcon;

Falcon::Falcon(int device_id, std::string canbus,
               std::vector<ctre::phoenixpro::BaseStatusSignalValue *> *signals,
               double stator_current_limit, double supply_current_limit)
    : talon_(device_id, canbus),
      device_id_(device_id),
      device_temp_(talon_.GetDeviceTemp()),
      supply_voltage_(talon_.GetSupplyVoltage()),
      supply_current_(talon_.GetSupplyCurrent()),
      torque_current_(talon_.GetTorqueCurrent()),
      position_(talon_.GetPosition()),
      duty_cycle_(talon_.GetDutyCycle()),
      stator_current_limit_(stator_current_limit),
      supply_current_limit_(supply_current_limit) {
  // device temp is not timesynced so don't add it to the list of signals
  device_temp_.SetUpdateFrequency(kCANUpdateFreqHz);

  CHECK_NOTNULL(signals);

  supply_voltage_.SetUpdateFrequency(kCANUpdateFreqHz);
  signals->push_back(&supply_voltage_);

  supply_current_.SetUpdateFrequency(kCANUpdateFreqHz);
  signals->push_back(&supply_current_);

  torque_current_.SetUpdateFrequency(kCANUpdateFreqHz);
  signals->push_back(&torque_current_);

  position_.SetUpdateFrequency(kCANUpdateFreqHz);
  signals->push_back(&position_);

  duty_cycle_.SetUpdateFrequency(kCANUpdateFreqHz);
  signals->push_back(&duty_cycle_);
}

void Falcon::PrintConfigs() {
  ctre::phoenixpro::configs::TalonFXConfiguration configuration;
  ctre::phoenix::StatusCode status =
      talon_.GetConfigurator().Refresh(configuration);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to get falcon configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }
  AOS_LOG(INFO, "configuration: %s", configuration.ToString().c_str());
}

void Falcon::WriteConfigs(ctre::phoenixpro::signals::InvertedValue invert) {
  inverted_ = invert;

  ctre::phoenixpro::configs::CurrentLimitsConfigs current_limits;
  current_limits.StatorCurrentLimit = stator_current_limit_;
  current_limits.StatorCurrentLimitEnable = true;
  current_limits.SupplyCurrentLimit = supply_current_limit_;
  current_limits.SupplyCurrentLimitEnable = true;

  ctre::phoenixpro::configs::MotorOutputConfigs output_configs;
  output_configs.NeutralMode =
      ctre::phoenixpro::signals::NeutralModeValue::Brake;
  output_configs.DutyCycleNeutralDeadband = 0;

  output_configs.Inverted = inverted_;

  ctre::phoenixpro::configs::TalonFXConfiguration configuration;
  configuration.CurrentLimits = current_limits;
  configuration.MotorOutput = output_configs;

  ctre::phoenix::StatusCode status =
      talon_.GetConfigurator().Apply(configuration);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to set falcon configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }

  PrintConfigs();
}

void Falcon::SerializePosition(flatbuffers::FlatBufferBuilder *fbb) {
  control_loops::CANFalcon::Builder builder(*fbb);
  builder.add_id(device_id_);
  builder.add_device_temp(device_temp());
  builder.add_supply_voltage(supply_voltage());
  builder.add_supply_current(supply_current());
  builder.add_torque_current(torque_current());
  builder.add_duty_cycle(duty_cycle());
  builder.add_position(position());

  last_position_offset_ = builder.Finish();
}

std::optional<flatbuffers::Offset<control_loops::CANFalcon>>
Falcon::TakeOffset() {
  auto option_offset = last_position_offset_;

  last_position_offset_.reset();

  return option_offset;
}
