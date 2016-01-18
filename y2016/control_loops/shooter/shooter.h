#ifndef Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
#define Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2016/control_loops/shooter/shooter_plant.h"
#include "y2016/control_loops/shooter/shooter.q.h"

namespace y2016 {
namespace control_loops {

namespace {
// TODO(constants): Update.
const double kTolerance = 10.0;
const double kMaxSpeed = 10000.0 * (2.0 * M_PI) / 60.0 * 15.0 / 34.0;
const double kAngularVelocityWeightScalar = 0.35;
}  // namespace

struct ShooterStatus {
  double avg_angular_velocity;
  bool ready;
};

class ShooterSide {
 public:
  ShooterSide();

  void SetGoal(double angular_velocity_goal);
  void EstimatePositionTimestep();
  void SetPosition(double current_position);
  const ShooterStatus GetStatus();
  double GetOutput();
  void UpdateLoop(bool output_is_null);

 private:
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> loop_;

  double current_position_ = 0.0;
  double position_goal_ = 0.0;
  double angular_velocity_goal_ = 0.0;

  // History array for calculating a filtered angular velocity.
  static const int kHistoryLength = 5;
  double history_[kHistoryLength];
  ptrdiff_t history_position_;

  DISALLOW_COPY_AND_ASSIGN(ShooterSide);
};

class Shooter
    : public ::aos::controls::ControlLoop<control_loops::ShooterQueue> {
 public:
  explicit Shooter(control_loops::ShooterQueue *shooter_queue =
                       &control_loops::shooter_queue);

 protected:
  void RunIteration(const control_loops::ShooterQueue::Goal *goal,
                    const control_loops::ShooterQueue::Position *position,
                    control_loops::ShooterQueue::Output *output,
                    control_loops::ShooterQueue::Status *status) override;

 private:
  ShooterSide left_, right_;

  DISALLOW_COPY_AND_ASSIGN(Shooter);
};

}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
