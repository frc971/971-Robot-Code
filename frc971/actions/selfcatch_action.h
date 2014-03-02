#include "frc971/actions/selfcatch_action.q.h"
#include "frc971/actions/action.h"
#include "aos/common/time.h"

namespace frc971 {
namespace actions {

class SelfCatchAction : public ActionBase<actions::SelfCatchActionGroup> {
 public:

  explicit SelfCatchAction(actions::SelfCatchActionGroup* s);

  // Actually execute the action of moving the claw and shooter into position
  // and actually firing them.
  void RunAction();

  // calc an offset to our requested shot based on robot speed
  double SpeedToAngleOffset(double speed);

  static constexpr double kSpeedOffsetRadians = 0.2;
  static constexpr double kShotPower = 100.0;
  static constexpr double kCatchSeperation = 1.0;
  static constexpr double kCatchIntake = 12.0;
  static constexpr double kSonarTriggerDist = 0.8;
  static constexpr double kCatchCentering = 12.0;
  static constexpr double kFinishAngle = 0.2;

 protected:
  // completed shot
  bool DoneShot();
  // ready for shot
  bool DonePreShotOpen();
  // in the right place
  bool DoneSetupShot();
  // sonar is in valid range to close
  bool DoneFoundSonar();
  // Claw reports it is done
  bool DoneClawWithBall();
  // hall effect reports the ball is in
  bool DoneBallIn();

  // to track when shot is complete
  int previous_shots_;
};

}  // namespace actions
}  // namespace frc971

