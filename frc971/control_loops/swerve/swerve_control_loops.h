#ifndef FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_

#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_output_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_position_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_static.h"

namespace frc971::control_loops::swerve {

inline void PopulateSwerveModuleRotation(
    SwerveModuleStatusStatic *swerve_module_table) {
  auto rotation = swerve_module_table->add_rotation();
  auto estimator_state = rotation->add_estimator_state();
  (void)estimator_state;
}

// Handles the translation and rotation current for each swerve module
class SwerveControlLoops
    : public ::frc971::controls::ControlLoop<Goal, Position, StatusStatic,
                                             OutputStatic> {
 public:
  explicit SwerveControlLoops(::aos::EventLoop *event_loop,
                              const ::std::string &name = "/swerve");

 protected:
  void RunIteration(
      const Goal *goal, const Position *position,
      aos::Sender<OutputStatic>::StaticBuilder *output_builder,
      aos::Sender<StatusStatic>::StaticBuilder *status_builder) override;
};

}  // namespace frc971::control_loops::swerve

#endif  // FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_
