#ifndef FRC971_WPILIB_SWERVE_DRIVETRAIN_WRITER_H_
#define FRC971_WPILIB_SWERVE_DRIVETRAIN_WRITER_H_

#include "ctre/phoenix6/TalonFX.hpp"

#include "frc971/can_configuration_generated.h"
#include "frc971/control_loops/drivetrain/swerve/swerve_drivetrain_output_generated.h"
#include "frc971/wpilib/falcon.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/swerve/swerve_module.h"

namespace frc971 {
namespace wpilib {
namespace swerve {

// Reads from the swerve output flatbuffer and uses wpilib to set the current
// for each motor.
class DrivetrainWriter
    : public ::frc971::wpilib::LoopOutputHandler<
          ::frc971::control_loops::drivetrain::swerve::Output> {
 public:
  DrivetrainWriter(::aos::EventLoop *event_loop, int drivetrain_writer_priority,
                   double max_voltage);

  void set_falcons(std::shared_ptr<SwerveModule> front_left,
                   std::shared_ptr<SwerveModule> front_right,
                   std::shared_ptr<SwerveModule> back_left,
                   std::shared_ptr<SwerveModule> back_right);

  void HandleCANConfiguration(const CANConfiguration &configuration);

 private:
  void WriteConfigs();

  void Write(const ::frc971::control_loops::drivetrain::swerve::Output &output)
      override;

  void Stop() override;

  double SafeSpeed(double voltage);

  std::shared_ptr<SwerveModule> front_left_, front_right_, back_left_,
      back_right_;

  double max_voltage_;
};

}  // namespace swerve
}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_SWERVE_DRIVETRAIN_WRITER_H_
