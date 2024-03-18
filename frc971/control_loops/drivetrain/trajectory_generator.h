#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_GENERATOR_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_GENERATOR_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/spline_goal_generated.h"
#include "frc971/control_loops/drivetrain/trajectory.h"

namespace frc971::control_loops::drivetrain {

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(aos::EventLoop *event_loop,
                      const DrivetrainConfig<double> &config);
  void HandleSplineGoal(const SplineGoal &goal);

 private:
  aos::EventLoop *const event_loop_;
  const DrivetrainConfig<double> dt_config_;
  aos::Sender<fb::Trajectory> trajectory_sender_;
  aos::Fetcher<SplineGoal> spline_goal_fetcher_;
};

}  // namespace frc971::control_loops::drivetrain

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_TRAJECTORY_GENERATOR_H_
