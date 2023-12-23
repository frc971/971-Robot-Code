#ifndef FRC971_WPILIB_FALCON_H_
#define FRC971_WPILIB_FALCON_H_

#include <chrono>
#include <cinttypes>
#include <vector>

#include "ctre/phoenix6/TalonFX.hpp"
#include "glog/logging.h"

#include "aos/commonmath.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/drivetrain_can_position_generated.h"

namespace control_loops = ::frc971::control_loops;

namespace frc971 {
namespace wpilib {

struct FalconParams {
  int device_id;
  bool inverted;
};

static constexpr units::frequency::hertz_t kCANUpdateFreqHz = 200_Hz;
static constexpr double kMaxBringupPower = 12.0;

// Gets info from and writes to falcon motors using the TalonFX controller.
class Falcon {
 public:
  Falcon(int device_id, bool inverted, std::string canbus,
         std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
         double stator_current_limit, double supply_current_limit);

  Falcon(FalconParams params, std::string canbus,
         std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
         double stator_current_limit, double supply_current_limit);

  void PrintConfigs();

  void WriteConfigs();
  ctre::phoenix::StatusCode WriteCurrent(double current, double max_voltage);

  ctre::phoenix::StatusCode WriteVoltage(double voltage);

  ctre::phoenix6::hardware::TalonFX *talon() { return &talon_; }

  // The position of the Falcon output shaft is multiplied by gear_ratio
  void SerializePosition(flatbuffers::FlatBufferBuilder *fbb,
                         double gear_ratio);

  std::optional<flatbuffers::Offset<control_loops::CANFalcon>> TakeOffset();

  int device_id() const { return device_id_; }
  float device_temp() const { return device_temp_.GetValue().value(); }
  float supply_voltage() const { return supply_voltage_.GetValue().value(); }
  float supply_current() const { return supply_current_.GetValue().value(); }
  float torque_current() const { return torque_current_.GetValue().value(); }
  float duty_cycle() const { return duty_cycle_.GetValue().value(); }
  float position() const {
    return static_cast<units::angle::radian_t>(position_.GetValue()).value();
  }

  // returns the monotonic timestamp of the latest timesynced reading in the
  // timebase of the the syncronized CAN bus clock.
  int64_t GetTimestamp() {
    std::chrono::nanoseconds latest_timestamp =
        torque_current_.GetTimestamp().GetTime();

    return latest_timestamp.count();
  }

  void RefreshNontimesyncedSignals() { device_temp_.Refresh(); };

  void set_stator_current_limit(double stator_current_limit) {
    stator_current_limit_ = stator_current_limit;
  }

  void set_supply_current_limit(double supply_current_limit) {
    supply_current_limit_ = supply_current_limit;
  }

  static double SafeSpeed(double voltage) {
    return (::aos::Clip(voltage, -kMaxBringupPower, kMaxBringupPower) / 12.0);
  }

 private:
  ctre::phoenix6::hardware::TalonFX talon_;
  int device_id_;

  ctre::phoenix6::signals::InvertedValue inverted_;

  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> device_temp_;
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> supply_voltage_;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> supply_current_,
      torque_current_;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> position_;
  ctre::phoenix6::StatusSignal<units::dimensionless::scalar_t> duty_cycle_;

  double stator_current_limit_;
  double supply_current_limit_;

  std::optional<flatbuffers::Offset<control_loops::CANFalcon>>
      last_position_offset_;
};
}  // namespace wpilib
}  // namespace frc971
#endif  // FRC971_WPILIB_FALCON_H_
