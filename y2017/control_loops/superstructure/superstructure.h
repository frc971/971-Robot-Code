#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2017/control_loops/superstructure/column/column.h"
#include "y2017/control_loops/superstructure/hood/hood.h"
#include "y2017/control_loops/superstructure/intake/intake.h"
#include "y2017/control_loops/superstructure/shooter/shooter.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"
#include "y2017/control_loops/superstructure/vision_distance_average.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);

  const hood::Hood &hood() const { return hood_; }
  const intake::Intake &intake() const { return intake_; }
  const shooter::Shooter &shooter() const { return shooter_; }
  const column::Column &column() const { return column_; }

  // Sets the ignore collisions bit.  This should *not* be used on the robot.
  void set_ignore_collisions(bool ignore_collisions) {
    ignore_collisions_ = ignore_collisions;
  }

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position *position,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  hood::Hood hood_;
  intake::Intake intake_;
  shooter::Shooter shooter_;
  column::Column column_;

  // If true, we ignore collisions.
  bool ignore_collisions_ = false;

  VisionDistanceAverage distance_average_;

  ::frc971::shooter_interpolation::InterpolationTable<
      ::y2017::constants::Values::ShotParams>
      shot_interpolation_table_;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
