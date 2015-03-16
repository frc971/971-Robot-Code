#ifndef FRC971_ACTORS_FRIDGE_PROFILE_LIB_H_
#define FRC971_ACTORS_FRIDGE_PROFILE_LIB_H_

#include <cmath>

#include "aos/common/controls/control_loop.h"
#include "aos/common/actions/actor.h"
#include "aos/common/util/phased_loop.h"
#include "frc971/control_loops/fridge/fridge.q.h"

namespace frc971 {
namespace actors {

struct ProfileParams {
  double velocity;
  double acceleration;
};

// Base class to provide helper utilities to all Actors who want to control the
// fridge.
template <typename T>
class FridgeActorBase : public aos::common::actions::ActorBase<T> {
 public:
  FridgeActorBase(T *queues) : aos::common::actions::ActorBase<T>(queues) {}

 protected:
  void DoFridgeProfile(double height, double angle,
                       ProfileParams elevator_parameters,
                       ProfileParams arm_parameters, bool grabbers) {
    DoFridgeProfile(height, angle, elevator_parameters, arm_parameters,
                    grabbers, grabbers, grabbers);
  }

  bool StartFridgeProfile(double height, double angle,
                          ProfileParams elevator_parameters,
                          ProfileParams arm_parameters, bool top_grabbers,
                          bool front_grabbers, bool back_grabbers) {
    auto new_fridge_goal = control_loops::fridge_queue.goal.MakeMessage();
    new_fridge_goal->profiling_type = 0;
    new_fridge_goal->max_velocity = elevator_parameters.velocity;
    new_fridge_goal->max_acceleration = elevator_parameters.acceleration;
    new_fridge_goal->height = height;
    new_fridge_goal->velocity = 0.0;
    new_fridge_goal->max_angular_velocity = arm_parameters.velocity;
    new_fridge_goal->max_angular_acceleration = arm_parameters.acceleration;
    new_fridge_goal->angle = angle;
    new_fridge_goal->angular_velocity = 0.0;
    new_fridge_goal->grabbers.top_front = top_grabbers;
    new_fridge_goal->grabbers.top_back = top_grabbers;
    new_fridge_goal->grabbers.bottom_front = front_grabbers;
    new_fridge_goal->grabbers.bottom_back = back_grabbers;
    LOG(INFO, "Starting profile to %f, %f\n", height, angle);

    if (!new_fridge_goal.Send()) {
      LOG(ERROR, "Failed to send fridge goal\n");
      return false;
    }
    return true;
  }

  enum ProfileStatus { RUNNING, DONE, CANCELING, CANCELED };

  ProfileStatus IterateProfile(double height, double angle,
                               ProfileParams elevator_parameters,
                               ProfileParams arm_parameters, bool top_grabbers,
                               bool front_grabbers, bool back_grabbers) {
    bool should_cancel = false;
    if (this->ShouldCancel()) {
      should_cancel = true;
      LOG(INFO, "Canceling fridge movement\n");
      auto new_fridge_goal = control_loops::fridge_queue.goal.MakeMessage();
      new_fridge_goal->profiling_type = 0;
      new_fridge_goal->max_velocity = elevator_parameters.velocity;
      new_fridge_goal->max_acceleration = elevator_parameters.acceleration;
      new_fridge_goal->height =
          control_loops::fridge_queue.status->height +
          (control_loops::fridge_queue.status->goal_velocity *
           ::std::abs(control_loops::fridge_queue.status->goal_velocity)) /
              (2.0 * new_fridge_goal->max_acceleration);
      height = new_fridge_goal->height;
      new_fridge_goal->velocity = 0.0;
      new_fridge_goal->max_angular_velocity = arm_parameters.velocity;
      new_fridge_goal->max_angular_acceleration = arm_parameters.acceleration;
      new_fridge_goal->angle =
          control_loops::fridge_queue.status->angle +
          (control_loops::fridge_queue.status->goal_angular_velocity *
           ::std::abs(
               control_loops::fridge_queue.status->goal_angular_velocity)) /
              (2.0 * new_fridge_goal->max_angular_acceleration);
      angle = new_fridge_goal->angle;
      new_fridge_goal->angular_velocity = 0.0;
      new_fridge_goal->grabbers.top_front = top_grabbers;
      new_fridge_goal->grabbers.top_back = top_grabbers;
      new_fridge_goal->grabbers.bottom_front = front_grabbers;
      new_fridge_goal->grabbers.bottom_back = back_grabbers;

      if (!new_fridge_goal.Send()) {
        LOG(ERROR, "Failed to send fridge goal\n");
        return CANCELED;
      }
    }
    control_loops::fridge_queue.status.FetchAnother();

    constexpr double kProfileError = 1e-5;
    constexpr double kAngleEpsilon = 0.02, kHeightEpsilon = 0.015;

    if (control_loops::fridge_queue.status->state != 4) {
      LOG(ERROR, "Fridge no longer running, aborting action\n");
      return CANCELED;
    }

    if (::std::abs(control_loops::fridge_queue.status->goal_angle - angle) <
            kProfileError &&
        ::std::abs(control_loops::fridge_queue.status->goal_height - height) <
            kProfileError &&
        ::std::abs(control_loops::fridge_queue.status->goal_angular_velocity) <
            kProfileError &&
        ::std::abs(control_loops::fridge_queue.status->goal_velocity) <
            kProfileError) {
      LOG(INFO, "Profile done.\n");
      if (should_cancel) {
        LOG(INFO, "Canceling.\n");
        return CANCELED;
      } else if (::std::abs(control_loops::fridge_queue.status->angle - angle) <
                     kAngleEpsilon &&
                 ::std::abs(control_loops::fridge_queue.status->height -
                            height) < kHeightEpsilon) {
        LOG(INFO, "Near goal, done.\n");
        return DONE;
      }
    }

    if (should_cancel) {
      return CANCELING;
    } else {
      return RUNNING;
    }
  }

  void DoFridgeProfile(double height, double angle,
                       ProfileParams elevator_parameters,
                       ProfileParams arm_parameters, bool top_grabbers,
                       bool front_grabbers, bool back_grabbers) {
    if (!StartFridgeProfile(height, angle, elevator_parameters, arm_parameters,
                            top_grabbers, front_grabbers, back_grabbers)) {
      return;
    }

    while (true) {
      ProfileStatus status =
          IterateProfile(height, angle, elevator_parameters, arm_parameters,
                         top_grabbers, front_grabbers, back_grabbers);
      if (status == DONE || status == CANCELED) {
        return;
      }
    }
  }

  bool WaitOrCancel(::aos::time::Time duration) {
    ::aos::time::Time end_time = ::aos::time::Time::Now() + duration;
    while (::aos::time::Time::Now() <= end_time) {
      ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(),
                                 2500);
      if (this->ShouldCancel()) return false;
    }
    return true;
  }
};

}  // namespace actors
}  // namespace frc971

#endif  // FRC971_ACTORS_FRIDGE_PROFILE_LIB_H_
