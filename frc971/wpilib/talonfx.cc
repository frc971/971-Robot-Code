#include "frc971/wpilib/talonfx.h"

using frc971::wpilib::kMaxBringupPower;
using frc971::wpilib::TalonFX;

TalonFX::TalonFX(int device_id, bool inverted, std::string canbus,
                 std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
                 double stator_current_limit, double supply_current_limit)
    : talon_(device_id, canbus),
      device_id_(device_id),
      neutral_mode_(ctre::phoenix6::signals::NeutralModeValue::Brake),
      inverted_(inverted),
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

  CHECK(signals != nullptr);

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

TalonFX::TalonFX(TalonFXParams params, std::string canbus,
                 std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
                 double stator_current_limit, double supply_current_limit)
    : TalonFX(params.device_id, params.inverted, canbus, signals,
              stator_current_limit, supply_current_limit) {}

void TalonFX::PrintConfigs() {
  ctre::phoenix6::configs::TalonFXConfiguration configuration;
  ctre::phoenix::StatusCode status =
      talon_.GetConfigurator().Refresh(configuration);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to get talonfx motor configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }
  AOS_LOG(INFO, "configuration: %s", configuration.ToString().c_str());
}

void TalonFX::WriteConfigs() {
  ctre::phoenix6::configs::CurrentLimitsConfigs current_limits;
  current_limits.StatorCurrentLimit =
      units::current::ampere_t{stator_current_limit_};
  current_limits.StatorCurrentLimitEnable = true;
  current_limits.SupplyCurrentLimit =
      units::current::ampere_t{supply_current_limit_};
  current_limits.SupplyCurrentLimitEnable = true;

  ctre::phoenix6::configs::MotorOutputConfigs output_configs;
  output_configs.NeutralMode = neutral_mode_;
  output_configs.DutyCycleNeutralDeadband = 0;

  output_configs.Inverted = inverted_;

  ctre::phoenix6::configs::TalonFXConfiguration configuration;
  configuration.CurrentLimits = current_limits;
  configuration.MotorOutput = output_configs;

  ctre::phoenix::StatusCode status =
      talon_.GetConfigurator().Apply(configuration);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to set talonfx motor configuration: %s: %s",
            status.GetName(), status.GetDescription());
  }

  PrintConfigs();
}

ctre::phoenix::StatusCode TalonFX::WriteCurrent(double current,
                                                double max_voltage) {
  ctre::phoenix6::controls::TorqueCurrentFOC control(
      static_cast<units::current::ampere_t>(current));
  // Using 0_Hz here makes it a one-shot update.
  control.UpdateFreqHz = 0_Hz;
  control.MaxAbsDutyCycle = SafeSpeed(max_voltage);
  ctre::phoenix::StatusCode status = talon()->SetControl(control);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to write control to talonfx motor %d: %s: %s",
            device_id(), status.GetName(), status.GetDescription());
  }

  return status;
}

ctre::phoenix::StatusCode TalonFX::WriteVoltage(double voltage,
                                                bool enable_foc) {
  ctre::phoenix6::controls::DutyCycleOut control(SafeSpeed(voltage));

  // Using 0_Hz here makes it a one-shot update.
  control.UpdateFreqHz = 0_Hz;
  control.EnableFOC = enable_foc;

  ctre::phoenix::StatusCode status = talon()->SetControl(control);
  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to write control to talonfx motor %d: %s: %s",
            device_id(), status.GetName(), status.GetDescription());
  }

  return status;
}
void TalonFX::SerializePosition(control_loops::CANTalonFXStatic *can_talonfx,
                                double gear_ratio) {
  can_talonfx->set_id(device_id_);
  can_talonfx->set_device_temp(device_temp());
  can_talonfx->set_supply_voltage(supply_voltage());
  can_talonfx->set_supply_current(supply_current());
  can_talonfx->set_torque_current(torque_current());
  can_talonfx->set_duty_cycle(duty_cycle());
  can_talonfx->set_position(position() * gear_ratio);
  can_talonfx->set_timestamp(GetTimestamp());
}
