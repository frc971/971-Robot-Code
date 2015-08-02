#ifndef Y2015_ACTORS_FRIDGE_PROFILE_LIB_H_
#define Y2015_ACTORS_FRIDGE_PROFILE_LIB_H_

#include <cmath>

#include "aos/common/actions/actor.h"
#include "aos/common/util/phased_loop.h"
#include "y2015/control_loops/fridge/fridge.q.h"

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

  enum ProfileStatus { RUNNING, DONE, CANCELED };

  ProfileStatus IterateProfile(double height, double angle,
                               ProfileParams elevator_parameters,
                               ProfileParams arm_parameters, bool top_grabbers,
                               bool front_grabbers, bool back_grabbers) {
    if (this->ShouldCancel()) {
      LOG(INFO, "Canceling fridge movement\n");
      if (!control_loops::fridge_queue.status.get()) {
        LOG(WARNING, "no fridge status so can't really cancel\n");
        return CANCELED;
      }

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
      }
      return CANCELED;
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
      if (::std::abs(control_loops::fridge_queue.status->angle - angle) <
                     kAngleEpsilon &&
                 ::std::abs(control_loops::fridge_queue.status->height -
                            height) < kHeightEpsilon) {
        LOG(INFO, "Near goal, done.\n");
        return DONE;
      }
    }

    return RUNNING;
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

  void DoFridgeXYProfile(double x, double y, ProfileParams x_parameters,
                         ProfileParams y_parameters, bool grabbers) {
    DoFridgeXYProfile(x, y, x_parameters, y_parameters, grabbers, grabbers,
                      grabbers);
  }

  void DoFridgeXYProfile(double x, double y, ProfileParams x_parameters,
                         ProfileParams y_parameters, bool top_grabbers,
                         bool front_grabbers, bool back_grabbers) {
    if (!StartFridgeXYProfile(x, y, x_parameters, y_parameters, top_grabbers,
                              front_grabbers, back_grabbers)) {
      return;
    }

    while (true) {
      ProfileStatus status =
          IterateXYProfile(x, y, x_parameters, y_parameters, top_grabbers,
                           front_grabbers, back_grabbers);
      if (status == DONE || status == CANCELED) {
        return;
      }
    }
  }

  void CancelXYMotion(ProfileParams x_parameters, ProfileParams y_parameters,
                      bool top_grabbers, bool front_grabbers,
                      bool back_grabbers) {
    LOG(INFO, "Canceling fridge movement\n");
    if (!control_loops::fridge_queue.status.get()) {
      LOG(WARNING, "no fridge status so can't really cancel\n");
      return;
    }

    auto new_fridge_goal = control_loops::fridge_queue.goal.MakeMessage();
    new_fridge_goal->profiling_type = 1;
    new_fridge_goal->max_x_velocity = x_parameters.velocity;
    new_fridge_goal->max_x_acceleration = x_parameters.acceleration;
    new_fridge_goal->x =
        control_loops::fridge_queue.status->x +
        (control_loops::fridge_queue.status->goal_x_velocity *
         ::std::abs(control_loops::fridge_queue.status->goal_x_velocity)) /
            (2.0 * new_fridge_goal->max_x_acceleration);
    new_fridge_goal->x_velocity = 0.0;

    new_fridge_goal->max_y_velocity = y_parameters.velocity;
    new_fridge_goal->max_y_acceleration = y_parameters.acceleration;
    new_fridge_goal->y =
        control_loops::fridge_queue.status->y +
        (control_loops::fridge_queue.status->goal_y_velocity *
         ::std::abs(control_loops::fridge_queue.status->goal_y_velocity)) /
            (2.0 * new_fridge_goal->max_y_acceleration);
    new_fridge_goal->y_velocity = 0.0;

    new_fridge_goal->grabbers.top_front = top_grabbers;
    new_fridge_goal->grabbers.top_back = top_grabbers;
    new_fridge_goal->grabbers.bottom_front = front_grabbers;
    new_fridge_goal->grabbers.bottom_back = back_grabbers;

    if (!new_fridge_goal.Send()) {
      LOG(ERROR, "Failed to send fridge goal\n");
    }
  }

  ProfileStatus IterateXYProfile(double x, double y, ProfileParams x_parameters,
                                 ProfileParams y_parameters, bool top_grabbers,
                                 bool front_grabbers, bool back_grabbers) {
    if (this->ShouldCancel()) {
      CancelXYMotion(x_parameters, y_parameters, top_grabbers, front_grabbers,
                     back_grabbers);
      return CANCELED;
    }
    control_loops::fridge_queue.status.FetchAnother();

    constexpr double kProfileError = 1e-5;
    constexpr double kXEpsilon = 0.02, kYEpsilon = 0.02;

    if (control_loops::fridge_queue.status->state != 4) {
      LOG(ERROR, "Fridge no longer running, aborting action\n");
      return CANCELED;
    }

    if (::std::abs(control_loops::fridge_queue.status->goal_x - x) <
            kProfileError &&
        ::std::abs(control_loops::fridge_queue.status->goal_y - y) <
            kProfileError &&
        ::std::abs(control_loops::fridge_queue.status->goal_x_velocity) <
            kProfileError &&
        ::std::abs(control_loops::fridge_queue.status->goal_y_velocity) <
            kProfileError) {
      LOG(INFO, "Profile done.\n");
      if (::std::abs(control_loops::fridge_queue.status->x - x) < kXEpsilon &&
          ::std::abs(control_loops::fridge_queue.status->y - y) < kYEpsilon) {
        LOG(INFO, "Near goal, done.\n");
        return DONE;
      }
    }

    return RUNNING;
  }

  bool StartFridgeXYProfile(double x, double y, ProfileParams x_parameters,
                            ProfileParams y_parameters, bool top_grabbers,
                            bool front_grabbers, bool back_grabbers) {
    auto new_fridge_goal = control_loops::fridge_queue.goal.MakeMessage();
    new_fridge_goal->profiling_type = 1;
    new_fridge_goal->max_x_velocity = x_parameters.velocity;
    new_fridge_goal->max_x_acceleration = x_parameters.acceleration;
    new_fridge_goal->x = x;
    new_fridge_goal->x_velocity = 0.0;

    new_fridge_goal->max_y_velocity = y_parameters.velocity;
    new_fridge_goal->max_y_acceleration = y_parameters.acceleration;
    new_fridge_goal->y = y;
    new_fridge_goal->y_velocity = 0.0;
    new_fridge_goal->grabbers.top_front = top_grabbers;
    new_fridge_goal->grabbers.top_back = top_grabbers;
    new_fridge_goal->grabbers.bottom_front = front_grabbers;
    new_fridge_goal->grabbers.bottom_back = back_grabbers;
    LOG(INFO, "Starting xy profile to %f, %f\n", x, y);

    if (!new_fridge_goal.Send()) {
      LOG(ERROR, "Failed to send fridge goal\n");
      return false;
    }
    return true;
  }
};

}  // namespace actors
}  // namespace frc971

#endif  // Y2015_ACTORS_FRIDGE_PROFILE_LIB_H_
