#include "frc971/actions/catch_action.q.h"
#include "frc971/actions/action.h"
#include "aos/common/time.h"

namespace frc971 {
namespace actions {

class CatchAction : public ActionBase<CatchActionGroup> {
 public:

  explicit CatchAction(CatchActionGroup* s);

  // Actually executes the action of moving the claw into position and closing
  // it.
  void RunAction();

  static constexpr double kCatchSeparation = 1.0;
  static constexpr double kCatchIntake = 12.0;
  static constexpr double kSonarTriggerDist = 0.8;
  static constexpr double kCatchCentering = 12.0;
  static constexpr double kFinishAngle = 0.2;

 protected:
  // ready for shot
  bool DonePreShotOpen();
  // in the right place
  bool DoneSetupCatch();
  // sonar is in valid range to close
  bool DoneFoundSonar();
  // Claw reports it is done
  bool DoneClawWithBall();
  // hall effect reports the ball is in
  bool DoneBallIn();
};

}  // namespace actions
}  // namespace frc971

