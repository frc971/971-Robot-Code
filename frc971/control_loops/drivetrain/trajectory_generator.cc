#include "frc971/control_loops/drivetrain/trajectory_generator.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

TrajectoryGenerator::TrajectoryGenerator(aos::EventLoop *event_loop,
                                         const DrivetrainConfig<double> &config)
    : event_loop_(event_loop),
      dt_config_(config),
      trajectory_sender_(
          event_loop_->MakeSender<fb::Trajectory>("/drivetrain")) {
  event_loop_->MakeWatcher("/drivetrain", [this](const SplineGoal &goal) {
    HandleSplineGoal(goal);
  });
}

void TrajectoryGenerator::HandleSplineGoal(const SplineGoal &goal) {
  Trajectory trajectory(goal, dt_config_);
  trajectory.Plan();

  aos::Sender<fb::Trajectory>::Builder builder =
      trajectory_sender_.MakeBuilder();

  CHECK_EQ(builder.Send(trajectory.Serialize(builder.fbb())),
           aos::RawSender::Error::kOk);
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
