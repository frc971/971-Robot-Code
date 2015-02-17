#ifndef FRC971_ACTIONS_FRIDGE_PROFILE_ACTION_H_
#define FRC971_ACTIONS_FRIDGE_PROFILE_ACTION_H_

#include <memory>

#include "frc971/actors/fridge_profile_action.q.h"
#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "aos/common/util/trapezoid_profile.h"

namespace frc971 {
namespace actors {

class FridgeProfileActor
    : public aos::common::actions::ActorBase<FridgeProfileActionQueueGroup> {
 public:
  explicit FridgeProfileActor(FridgeProfileActionQueueGroup *s);

  // sets up profiles. Returns false if things have already been setup
  bool InitializeProfile(double angle_max_vel, double angle_max_accel,
                         double height_max_vel, double height_max_accel);

  // Takes a goal and computes the next step toward that goal. Returns false if
  // things are broken.
  bool IterateProfile(double goal_angle, double goal_height, double *next_angle,
                      double *next_height, double *next_angle_velocity,
                      double *next_angle_accel);

  bool RunAction(const FridgeProfileParams &params) override;

  // only for unit test
  void SetTesting() { testing_ = true; }

 private:
  ::std::unique_ptr<aos::util::TrapezoidProfile> arm_profile_;
  ::std::unique_ptr<aos::util::TrapezoidProfile> elevator_profile_;
  double arm_start_angle_ = 0.0;
  double elev_start_height_ = 0.0;
  bool testing_ = false;
};

typedef aos::common::actions::TypedAction<FridgeProfileActionQueueGroup>
    FridgeAction;

// Makes a new FridgeProfileActor action.
::std::unique_ptr<FridgeAction> MakeFridgeProfileAction(
    const FridgeProfileParams &fridge_params);

}  // namespace actors
}  // namespace frc971

#endif
