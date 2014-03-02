//#include "aos/common/control_loop/Timing.h"
//#include "aos/common/logging/logging.h"

#include "frc971/actions/shoot_action.q.h"
#include "frc971/actions/action.h"

//#include "frc971/control_loops/shooter/shooter.q.h"
//#include "frc971/control_loops/claw/claw.q.h"
//#include "frc971/constants.h"

namespace frc971 {
namespace actions {

class ShootAction : public ActionBase<actions::ShootActionQueueGroup> {
 public:

  explicit ShootAction(actions::ShootActionQueueGroup* s);

  // Actually execute the action of moving the claw and shooter into position
  // and actually firing them.
  void RunAction();

  // calc an offset to our requested shot based on robot speed
  double SpeedToAngleOffset(double speed);

  static constexpr double kOffsetRadians = 0.2;

 protected:
};

}  // namespace actions
}  // namespace frc971

