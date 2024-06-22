#ifndef FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_

#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/control_loops/profiled_subsystem_static.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_output_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_position_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_static.h"
#include "frc971/control_loops/swerve/swerve_zeroing_static.h"
#include "frc971/zeroing/continuous_absolute_encoder.h"

namespace frc971::control_loops::swerve {

inline void PopulateSwerveModuleRotation(
    SwerveModuleStatusStatic *swerve_module_table,
    const frc971::control_loops::AbsoluteEncoderProfiledJointStatus
        *rotation_status) {
  auto rotation = swerve_module_table->add_rotation();
  CHECK(rotation->FromFlatbuffer(rotation_status));
}

// Handles the translation and rotation current for each swerve module
class SwerveControlLoops
    : public ::frc971::controls::ControlLoop<Goal, Position, StatusStatic,
                                             OutputStatic> {
 public:
  using AbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::ContinuousAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

  explicit SwerveControlLoops(
      ::aos::EventLoop *event_loop,
      const frc971::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemCommonParams *rotation_params,
      const SwerveZeroing *zeroing_params,
      const ::std::string &name = "/swerve");

 protected:
  void RunIteration(
      const Goal *goal, const Position *position,
      aos::Sender<OutputStatic>::StaticBuilder *output_builder,
      aos::Sender<StatusStatic>::StaticBuilder *status_builder) override;
  AbsoluteEncoderSubsystem front_left_, front_right_, back_left_, back_right_;
};

}  // namespace frc971::control_loops::swerve

#endif  // FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_
