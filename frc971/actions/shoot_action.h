#include "frc971/actions/shoot_action.q.h"
#include "frc971/actions/action.h"

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
  // completed shot
  bool DoneShot();
  // ready for shot
  bool DonePreShotOpen();
  // in the right place
  bool DoneSetupShot();

  // to track when shot is complete
  int previous_shots_;
};

}  // namespace actions
}  // namespace frc971

