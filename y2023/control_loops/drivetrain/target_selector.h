#ifndef Y2023_CONTROL_LOOPS_DRIVETRAIN_TARGET_SELECTOR_H_
#define Y2023_CONTROL_LOOPS_DRIVETRAIN_TARGET_SELECTOR_H_
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/localizer.h"
#include "frc971/control_loops/pose.h"
#include "frc971/input/joystick_state_generated.h"
#include "y2023/constants/constants_generated.h"
#include "y2023/control_loops/drivetrain/target_selector_hint_generated.h"
#include "y2023/control_loops/drivetrain/target_selector_status_generated.h"

namespace y2023::control_loops::drivetrain {
// This target selector provides the logic to choose which position to try to
// guide the robot to (primarily for game piece placement; but also for game
// piece pickup).
// Currently, this works by:
// 1. Relying on the constants + JoystickState message to figure out which set
//    of targets are relevant to us given the alliance that we are on).
// 2. If the TargetSelectorHint message fully specifies where to score the game
//    piece, go there.
// 3. If the exact grid to score in is unpopulated, score in the closest grid.
//    In the future, the code could readily be expanded to score in the nearest
//    valid position or resolve any other set of extra ambiguity.
class TargetSelector
    : public frc971::control_loops::drivetrain::TargetSelectorInterface {
 public:
  typedef frc971::control_loops::TypedPose<double> Pose;
  typedef frc971::control_loops::drivetrain::RobotSide Side;

  TargetSelector(aos::EventLoop *event_loop);

  bool UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &state,
                       double command_speed) override;

  Pose TargetPose() const override {
    CHECK(target_pose_.has_value())
        << "Did you check the return value of UpdateSelection()?";
    return target_pose_.value();
  }

  double TargetRadius() const override { return 0.0; }
  double GamePieceRadius() const override { return game_piece_position_; }
  bool SignedRadii() const override { return true; }
  Side DriveDirection() const override { return drive_direction_; }
  // We will manage any desired hysteresis in the target selection.
  bool ForceReselectTarget() const override { return true; }

 private:
  void UpdateAlliance();
  // Returns the Y coordinate of a game piece given the time-of-flight reading.
  double LateralOffsetForTimeOfFlight(double reading) const;
  std::optional<Pose> target_pose_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<TargetSelectorHint> hint_fetcher_;
  aos::Sender<TargetSelectorStatus> status_sender_;
  std::optional<TargetSelectorHintT> last_hint_;
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const localizer::HalfField *scoring_map_ = nullptr;
  double game_piece_position_ = 0.0;
  Side drive_direction_ = Side::DONT_CARE;
};
}  // namespace y2023::control_loops::drivetrain
#endif  // Y2023_CONTROL_LOOPS_DRIVETRAIN_TARGET_SELECTOR_H_
