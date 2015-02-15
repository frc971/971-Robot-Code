#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/common/commonmath.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/actions/actor.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/trapezoid_profile.h"

#include "frc971/constants.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/control_loops/fridge/fridge.q.h"

namespace frc971 {
namespace actors {

FridgeProfileActor::FridgeProfileActor(actors::FridgeProfileActionQueueGroup* s)
    : aos::common::actions::ActorBase<actors::FridgeProfileActionQueueGroup>(s) {}

bool FridgeProfileActor::InitializeProfile(double angle_max_vel,
                                           double angle_max_accel,
                                           double height_max_vel,
                                           double height_max_accel) {
  if (arm_profile_ != nullptr || elevator_profile_ != nullptr) {
    return false;
  }
  // Initialize arm profile.
  arm_profile_.reset(
      new ::aos::util::TrapezoidProfile(::aos::time::Time::InMS(5)));
  arm_profile_->set_maximum_velocity(angle_max_vel);
  arm_profile_->set_maximum_acceleration(angle_max_accel);

  // Initialize elevator profile.
  elevator_profile_.reset(
      new ::aos::util::TrapezoidProfile(::aos::time::Time::InMS(5)));
  elevator_profile_->set_maximum_velocity(height_max_vel);
  elevator_profile_->set_maximum_acceleration(height_max_accel);
  return true;
}

bool FridgeProfileActor::IterateProfile(double goal_angle, double goal_height,
                                        double* next_angle,
                                        double* next_height,
                                        double* next_angle_velocity,
                                        double* next_height_velocity) {
  ::Eigen::Matrix<double, 2, 1> goal_state;

  goal_state = arm_profile_->Update(goal_angle, 0.0);
  *next_angle = goal_state(0, 0);
  *next_angle_velocity = goal_state(1, 0);
  goal_state = elevator_profile_->Update(goal_height, 0.0);
  *next_height = goal_state(0, 0);
  *next_height_velocity = goal_state(1, 0);

  return true;
}

bool FridgeProfileActor::RunAction() {
  double goal_angle = action_q_->goal->arm_angle;
  double goal_height = action_q_->goal->elevator_height;
  bool top_front = action_q_->goal->top_front_grabber;
  bool top_back = action_q_->goal->top_back_grabber;
  bool bottom_front = action_q_->goal->bottom_front_grabber;
  bool bottom_back = action_q_->goal->bottom_back_grabber;
  LOG(INFO,
      "Fridge profile goal: arm (%f) elev (%f) with grabbers(%d,%d,%d,%d).\n",
      goal_angle, goal_height, top_front, top_back, bottom_front, bottom_back);

  // defines finished
  const double angle_epsilon = 0.01, height_epsilon = 0.01;

  // Initialize arm profile.
  if(!InitializeProfile(action_q_->goal->arm_max_velocity,
                   action_q_->goal->arm_max_acceleration,
                   action_q_->goal->elevator_max_velocity,
                   action_q_->goal->elevator_max_acceleration)) {
    return false;
  }

  control_loops::fridge_queue.status.FetchLatest();
  if (control_loops::fridge_queue.status.get()) {
    if (!control_loops::fridge_queue.status->zeroed) {
      LOG(ERROR, "We are not running actions on an unzeroed fridge!\n");
      return false;
    }
    arm_start_angle_ = control_loops::fridge_queue.status->angle;
    elev_start_height_ = control_loops::fridge_queue.status->height;
  } else {
    LOG(ERROR, "No fridge status!\n");
    return false;
  }

  while (true) {
    // wait until next Xms tick
    ::aos::time::PhasedLoopXMS(5, 2500);

    double delta_angle, delta_height;
    double angle_vel, height_vel;
    if (!IterateProfile(goal_angle, goal_height, &delta_angle, &delta_height,
        &angle_vel, &height_vel)) {
      return false;
    }

    // check if we should stop before we send
    if (ShouldCancel()) return true;

    auto message = control_loops::fridge_queue.goal.MakeMessage();
    message->angle = arm_start_angle_ + delta_angle;
    message->angular_velocity = angle_vel;
    message->height = elev_start_height_ + delta_height;
    message->velocity = height_vel;
    message->grabbers.top_front = top_front;
    message->grabbers.top_back = top_back;
    message->grabbers.bottom_front = top_front;
    message->grabbers.top_front = top_front;

    LOG_STRUCT(DEBUG, "Sending fridge goal", *message);
    message.Send();

    control_loops::fridge_queue.status.FetchLatest();
    if (!control_loops::fridge_queue.status.get()) {
      return false;
    }
    const double current_height = control_loops::fridge_queue.status->height;
    const double current_angle = control_loops::fridge_queue.status->angle;
    LOG_STRUCT(DEBUG, "Got fridge status",
               *control_loops::fridge_queue.status);

    if (::std::abs(arm_start_angle_ + delta_angle - goal_angle) <
            angle_epsilon &&
        ::std::abs(arm_start_angle_ + delta_angle - current_angle) <
            angle_epsilon &&
        ::std::abs(elev_start_height_ + delta_height - goal_height) <
            height_epsilon &&
        ::std::abs(elev_start_height_ + delta_height - current_height) <
            height_epsilon) {
      break;
    }
  }

  arm_profile_.reset();
  arm_profile_.reset();
  arm_start_angle_ = 0.0;
  elev_start_height_ = 0.0;

  LOG(INFO, "Fridge profile done moving.\n");
  return true;
}

::std::unique_ptr<aos::common::actions::TypedAction<
    ::frc971::actors::FridgeProfileActionQueueGroup>>
MakeFridgeProfileAction() {
  return ::std::unique_ptr<aos::common::actions::TypedAction<
      ::frc971::actors::FridgeProfileActionQueueGroup>>(
      new aos::common::actions::TypedAction<
          ::frc971::actors::FridgeProfileActionQueueGroup>(
          &::frc971::actors::fridge_profile_action));
}

}  // namespace actors
}  // namespace frc971
