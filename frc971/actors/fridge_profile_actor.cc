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
    : aos::common::actions::ActorBase<actors::FridgeProfileActionQueueGroup>(
          s) {}

bool FridgeProfileActor::InitializeProfile(double angle_max_vel,
                                           double angle_max_accel,
                                           double height_max_vel,
                                           double height_max_accel) {
  // if they have no vel/accel they will not move
  if (angle_max_vel != 0.0 && angle_max_accel != 0.0) {
    // Initialize arm profile.
    arm_profile_.reset(
        new ::aos::util::TrapezoidProfile(::aos::time::Time::InMS(5)));
    arm_profile_->set_maximum_velocity(angle_max_vel);
    arm_profile_->set_maximum_acceleration(angle_max_accel);
  } else {
    arm_profile_.reset();
  }

  // if they have no vel/accel they will not move
  if (height_max_vel != 0.0 && height_max_accel != 0.0) {
    // Initialize elevator profile.
    elevator_profile_.reset(
        new ::aos::util::TrapezoidProfile(::aos::time::Time::InMS(5)));
    elevator_profile_->set_maximum_velocity(height_max_vel);
    elevator_profile_->set_maximum_acceleration(height_max_accel);
  } else {
    elevator_profile_.reset();
  }

  return true;
}

bool FridgeProfileActor::IterateProfile(double goal_angle, double goal_height,
                                        double* next_angle, double* next_height,
                                        double* next_angle_velocity,
                                        double* next_height_velocity) {
  CHECK_NOTNULL(next_angle);
  CHECK_NOTNULL(next_height);
  CHECK_NOTNULL(next_angle_velocity);
  CHECK_NOTNULL(next_height_velocity);
  ::Eigen::Matrix<double, 2, 1> goal_state;

  if (!arm_profile_) {
    *next_angle = arm_start_angle_;
    *next_angle_velocity = 0.0;
  } else {
    goal_state = arm_profile_->Update(goal_angle, 0.0);
    *next_angle = goal_state(0, 0);
    *next_angle_velocity = goal_state(1, 0);
  }

  if (!elevator_profile_) {
    *next_height = elev_start_height_;
    *next_height_velocity = 0.0;
  } else {
    goal_state = elevator_profile_->Update(goal_height, 0.0);
    *next_height = goal_state(0, 0);
    *next_height_velocity = goal_state(1, 0);
  }

  return true;
}

bool FridgeProfileActor::RunAction(const FridgeProfileParams &params) {
  const double goal_angle = params.arm_angle;
  const double goal_height = params.elevator_height;
  bool top_front = params.top_front_grabber;
  bool top_back = params.top_back_grabber;
  bool bottom_front = params.bottom_front_grabber;
  bool bottom_back = params.bottom_back_grabber;
  LOG(INFO,
      "Fridge profile goal: arm (%f) elev (%f) with grabbers(%d,%d,%d,%d).\n",
      goal_angle, goal_height, top_front, top_back, bottom_front, bottom_back);

  // defines finished
  constexpr double kAngleEpsilon = 0.02, kHeightEpsilon = 0.015;
  constexpr double kAngleProfileEpsilon = 0.0001, kHeightProfileEpsilon = 0.0001;

  // Initialize arm profile.
  if (!InitializeProfile(params.arm_max_velocity, params.arm_max_acceleration,
                         params.elevator_max_velocity,
                         params.elevator_max_acceleration)) {
    return false;
  }

  control_loops::fridge_queue.status.FetchLatest();
  if (control_loops::fridge_queue.status.get()) {
    if (!control_loops::fridge_queue.status->zeroed) {
      LOG(ERROR, "We are not running actions on an unzeroed fridge!\n");
      return false;
    }
    arm_start_angle_ = control_loops::fridge_queue.status->goal_angle;
    elev_start_height_ = control_loops::fridge_queue.status->goal_height;
  } else {
    LOG(ERROR, "No fridge status!\n");
    return false;
  }

  if (arm_profile_) {
    Eigen::Matrix<double, 2, 1> arm_current;
    arm_current.setZero();
    arm_current << arm_start_angle_, 0.0;
    arm_profile_->MoveCurrentState(arm_current);
  }
  if (elevator_profile_) {
    Eigen::Matrix<double, 2, 1> elevator_current;
    elevator_current.setZero();
    elevator_current << elev_start_height_, 0.0;
    elevator_profile_->MoveCurrentState(elevator_current);
  }

  while (true) {
    // wait until next Xms tick
    ::aos::time::PhasedLoopXMS(5, 2500);

    double current_goal_angle, current_goal_height;
    double angle_vel, height_vel;
    if (!IterateProfile(goal_angle, goal_height, &current_goal_angle,
                        &current_goal_height, &angle_vel, &height_vel)) {
      return false;
    }

    // check if we should stop before we send
    if (ShouldCancel()) return true;

    auto message = control_loops::fridge_queue.goal.MakeMessage();
    message->angle = current_goal_angle;
    message->angular_velocity = angle_vel;
    message->height = current_goal_height;
    message->velocity = height_vel;
    message->grabbers.top_front = top_front;
    message->grabbers.top_back = top_back;
    message->grabbers.bottom_front = bottom_front;
    message->grabbers.bottom_back = bottom_back;

    LOG_STRUCT(DEBUG, "Sending fridge goal", *message);
    message.Send();

    control_loops::fridge_queue.status.FetchLatest();
    if (!control_loops::fridge_queue.status.get()) {
      return false;
    }
    double current_angle = control_loops::fridge_queue.status->angle;
    double current_height = control_loops::fridge_queue.status->height;
    LOG_STRUCT(DEBUG, "Got fridge status", *control_loops::fridge_queue.status);

    if (testing_) {
      current_angle = goal_angle;
      current_height = goal_height;
    }

    const double arm_error = ::std::abs(current_goal_angle - current_angle);
    const double profile_error_arm =
        ::std::abs(current_goal_angle - goal_angle);

    const double profile_error_elevator =
        ::std::abs(current_goal_height - goal_height);
    const double elevator_error =
        ::std::abs(current_goal_height - current_height);

    if ((!arm_profile_ || (arm_error < kAngleEpsilon &&
                           profile_error_arm < kAngleProfileEpsilon)) &&
        (!elevator_profile_ ||
         (elevator_error < kHeightEpsilon &&
          profile_error_elevator < kHeightProfileEpsilon))) {
      break;
    } else {
      ErrorToLog error_to_log;
      error_to_log.arm_error = arm_error;
      error_to_log.profile_error_arm = profile_error_arm;
      error_to_log.profile_error_elevator = profile_error_elevator;
      error_to_log.elevator_error = elevator_error;
      LOG_STRUCT(DEBUG, "error", error_to_log);
    }
  }

  arm_profile_.reset();
  arm_profile_.reset();
  arm_start_angle_ = 0.0;
  elev_start_height_ = 0.0;

  LOG(INFO, "Fridge profile done moving.\n");
  return true;
}

::std::unique_ptr<FridgeAction> MakeFridgeProfileAction(
    const FridgeProfileParams& p) {
  return ::std::unique_ptr<FridgeAction>(
      new FridgeAction(&::frc971::actors::fridge_profile_action, p));
}

}  // namespace actors
}  // namespace frc971
