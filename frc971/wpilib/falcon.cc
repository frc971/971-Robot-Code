#include "frc971/wpilib/falcon.h"

using frc971::wpilib::Falcon;
using frc971::wpilib::kMaxBringupPower;

Falcon::Falcon(int device_id, std::string canbus,
               std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
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
  ctre::phoenix6::configs::TalonFXConfiguration configuration;
  ctre::phoenix::StatusCode status =
      talon_.GetConfigurator().Refresh(configuration);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to get falcon configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }
  AOS_LOG(INFO, "configuration: %s", configuration.ToString().c_str());
}

void Falcon::WriteConfigs(ctre::phoenix6::signals::InvertedValue invert) {
  inverted_ = invert;

  ctre::phoenix6::configs::CurrentLimitsConfigs current_limits;
  current_limits.StatorCurrentLimit = stator_current_limit_;
  current_limits.StatorCurrentLimitEnable = true;
  current_limits.SupplyCurrentLimit = supply_current_limit_;
  current_limits.SupplyCurrentLimitEnable = true;

  ctre::phoenix6::configs::MotorOutputConfigs output_configs;
  output_configs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  output_configs.DutyCycleNeutralDeadband = 0;

  output_configs.Inverted = inverted_;

  ctre::phoenix6::configs::TalonFXConfiguration configuration;
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

ctre::phoenix::StatusCode Falcon::WriteCurrent(double current,
                                               double max_voltage) {
  ctre::phoenix6::controls::TorqueCurrentFOC control(
      static_cast<units::current::ampere_t>(current));
  // Using 0_Hz here makes it a one-shot update.
  control.UpdateFreqHz = 0_Hz;
  control.MaxAbsDutyCycle =
      ::aos::Clip(max_voltage, -kMaxBringupPower, kMaxBringupPower) / 12.0;
  ctre::phoenix::StatusCode status = talon()->SetControl(control);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to write control to falcon %d: %s: %s", device_id(),
            status.GetName(), status.GetDescription());
  }

  return status;
}

void Falcon::SerializePosition(flatbuffers::FlatBufferBuilder *fbb,
                               double gear_ratio) {
  control_loops::CANFalcon::Builder builder(*fbb);
  builder.add_id(device_id_);
  builder.add_device_temp(device_temp());
  builder.add_supply_voltage(supply_voltage());
  builder.add_supply_current(supply_current());
  builder.add_torque_current(torque_current());
  builder.add_duty_cycle(duty_cycle());
  builder.add_position(position() * gear_ratio);

  last_position_offset_ = builder.Finish();
}

std::optional<flatbuffers::Offset<control_loops::CANFalcon>>
Falcon::TakeOffset() {
  auto option_offset = last_position_offset_;

  last_position_offset_.reset();

  return option_offset;
}
