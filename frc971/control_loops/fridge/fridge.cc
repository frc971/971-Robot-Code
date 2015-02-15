#include "frc971/control_loops/fridge/fridge.h"

#include <cmath>

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/fridge/elevator_motor_plant.h"
#include "frc971/control_loops/fridge/arm_motor_plant.h"
#include "frc971/zeroing/zeroing.h"

#include "frc971/constants.h"

namespace frc971 {
namespace control_loops {

constexpr double Fridge::dt;

namespace {
constexpr double kZeroingVoltage = 5.0;
constexpr double kElevatorZeroingVelocity = 0.1;
constexpr double kArmZeroingVelocity = 0.2;
}  // namespace


void CappedStateFeedbackLoop::CapU() {
  // TODO(austin): Use Campbell's code.
  if (mutable_U(0, 0) > max_voltage_) {
    mutable_U(0, 0) = max_voltage_;
  }
  if (mutable_U(1, 0) > max_voltage_) {
    mutable_U(1, 0) = max_voltage_;
  }
  if (mutable_U(0, 0) < -max_voltage_) {
    mutable_U(0, 0) = -max_voltage_;
  }
  if (mutable_U(1, 0) < -max_voltage_) {
    mutable_U(1, 0) = -max_voltage_;
  }
}

Eigen::Matrix<double, 2, 1>
CappedStateFeedbackLoop::UnsaturateOutputGoalChange() {
  // Compute the K matrix used to compensate for position errors.
  Eigen::Matrix<double, 2, 2> Kp;
  Kp.setZero();
  Kp.col(0) = K().col(0);
  Kp.col(1) = K().col(2);

  Eigen::Matrix<double, 2, 2> Kp_inv = Kp.inverse();

  // Compute how much we need to change R in order to achieve the change in U
  // that was observed.
  Eigen::Matrix<double, 2, 1> deltaR = -Kp_inv * (U_uncapped() - U());
  return deltaR;
}

Fridge::Fridge(control_loops::FridgeQueue *fridge)
    : aos::controls::ControlLoop<control_loops::FridgeQueue>(fridge),
      arm_loop_(new CappedStateFeedbackLoop(
          StateFeedbackLoop<4, 2, 2>(MakeArmLoop()))),
      elevator_loop_(new CappedStateFeedbackLoop(
          StateFeedbackLoop<4, 2, 2>(MakeElevatorLoop()))),
      left_arm_estimator_(constants::GetValues().fridge.left_arm_zeroing),
      right_arm_estimator_(constants::GetValues().fridge.right_arm_zeroing),
      left_elevator_estimator_(
          constants::GetValues().fridge.left_elev_zeroing),
      right_elevator_estimator_(
          constants::GetValues().fridge.right_elev_zeroing) {}

void Fridge::UpdateZeroingState() {
  if (left_elevator_estimator_.offset_ratio_ready() < 1.0 ||
      right_elevator_estimator_.offset_ratio_ready() < 1.0 ||
      left_arm_estimator_.offset_ratio_ready() < 1.0 ||
      right_arm_estimator_.offset_ratio_ready() < 1.0) {
    state_ = INITIALIZING;
  } else if (!left_elevator_estimator_.zeroed() ||
      !right_elevator_estimator_.zeroed()) {
    state_ = ZEROING_ELEVATOR;
  } else if (!left_arm_estimator_.zeroed() ||
      !right_arm_estimator_.zeroed()) {
    state_ = ZEROING_ARM;
  } else {
    state_ = RUNNING;
  }
}

void Fridge::Correct() {
  {
    Eigen::Matrix<double, 2, 1> Y;
    Y << left_elevator(), right_elevator();
    elevator_loop_->Correct(Y);
  }

  {
    Eigen::Matrix<double, 2, 1> Y;
    Y << left_arm(), right_arm();
    arm_loop_->Correct(Y);
  }
}

void Fridge::SetElevatorOffset(double left_offset, double right_offset) {
  LOG(INFO, "Changing Elevator offset from %f, %f to %f, %f\n",
      left_elevator_offset_, right_elevator_offset_, left_offset, right_offset);
  double left_doffset = left_offset - left_elevator_offset_;
  double right_doffset = right_offset - right_elevator_offset_;

  // Adjust the average height and height difference between the two sides.
  // The derivatives of both should not need to be updated since the speeds
  // haven't changed.
  // The height difference is calculated as left - right, not right - left.
  elevator_loop_->mutable_X_hat(0, 0) += (left_doffset + right_doffset) / 2;
  elevator_loop_->mutable_X_hat(2, 0) += (left_doffset - right_doffset) / 2;

  // Modify the zeroing goal.
  elevator_goal_ += (left_doffset + right_doffset) / 2;

  // Update the cached offset values to the actual values.
  left_elevator_offset_ = left_offset;
  right_elevator_offset_ = right_offset;
}

void Fridge::SetArmOffset(double left_offset, double right_offset) {
  LOG(INFO, "Changing Arm offset from %f, %f to %f, %f\n", left_arm_offset_,
      right_arm_offset_, left_offset, right_offset);
  double left_doffset = left_offset - left_arm_offset_;
  double right_doffset = right_offset - right_arm_offset_;

  // Adjust the average angle and angle difference between the two sides.
  // The derivatives of both should not need to be updated since the speeds
  // haven't changed.
  arm_loop_->mutable_X_hat(0, 0) += (left_doffset + right_doffset) / 2;
  arm_loop_->mutable_X_hat(2, 0) += (left_doffset - right_doffset) / 2;

  // Modify the zeroing goal.
  arm_goal_ += (left_doffset + right_doffset) / 2;

  // Update the cached offset values to the actual values.
  left_arm_offset_ = left_offset;
  right_arm_offset_ = right_offset;
}

double Fridge::estimated_left_elevator() {
  return current_position_.elevator.left.encoder +
         left_elevator_estimator_.offset();
}
double Fridge::estimated_right_elevator() {
  return current_position_.elevator.right.encoder +
         right_elevator_estimator_.offset();
}

double Fridge::estimated_elevator() {
  return (estimated_left_elevator() + estimated_right_elevator()) / 2.0;
}

double Fridge::estimated_left_arm() {
  return current_position_.elevator.left.encoder + left_arm_estimator_.offset();
}
double Fridge::estimated_right_arm() {
  return current_position_.elevator.right.encoder +
         right_arm_estimator_.offset();
}
double Fridge::estimated_arm() {
  return (estimated_left_arm() + estimated_right_arm()) / 2.0;
}

double Fridge::left_elevator() {
  return current_position_.elevator.left.encoder + left_elevator_offset_;
}
double Fridge::right_elevator() {
  return current_position_.elevator.right.encoder + right_elevator_offset_;
}

double Fridge::elevator() { return (left_elevator() + right_elevator()) / 2.0; }

double Fridge::left_arm() {
  return current_position_.arm.left.encoder + left_arm_offset_;
}
double Fridge::right_arm() {
  return current_position_.arm.right.encoder + right_arm_offset_;
}
double Fridge::arm() { return (left_arm() + right_arm()) / 2.0; }

double Fridge::elevator_zeroing_velocity() {
  double average_elevator =
      (constants::GetValues().fridge.elevator.lower_limit +
       constants::GetValues().fridge.elevator.upper_limit) /
      2.0;

  const double pulse_width = ::std::max(
      constants::GetValues().fridge.left_elev_zeroing.index_difference,
      constants::GetValues().fridge.right_elev_zeroing.index_difference);

  if (elevator_zeroing_velocity_ == 0) {
    if (estimated_elevator() > average_elevator) {
      elevator_zeroing_velocity_ = -kElevatorZeroingVelocity;
    } else {
      elevator_zeroing_velocity_ = kElevatorZeroingVelocity;
    }
  } else if (elevator_zeroing_velocity_ > 0 &&
             estimated_elevator() > average_elevator + 2 * pulse_width) {
    elevator_zeroing_velocity_ = -kElevatorZeroingVelocity;
  } else if (elevator_zeroing_velocity_ < 0 &&
             estimated_elevator() < average_elevator - 2 * pulse_width) {
    elevator_zeroing_velocity_ = kElevatorZeroingVelocity;
  }
  return elevator_zeroing_velocity_;
}

double Fridge::arm_zeroing_velocity() {
  const double average_arm = (constants::GetValues().fridge.arm.lower_limit +
                              constants::GetValues().fridge.arm.upper_limit) /
                             2.0;
  const double pulse_width = ::std::max(
      constants::GetValues().fridge.right_arm_zeroing.index_difference,
      constants::GetValues().fridge.left_arm_zeroing.index_difference);

  if (arm_zeroing_velocity_ == 0) {
    if (estimated_arm() > average_arm) {
      arm_zeroing_velocity_ = -kArmZeroingVelocity;
    } else {
      arm_zeroing_velocity_ = kArmZeroingVelocity;
    }
  } else if (arm_zeroing_velocity_ > 0.0 &&
             estimated_arm() > average_arm + 2.0 * pulse_width) {
    arm_zeroing_velocity_ = -kArmZeroingVelocity;
  } else if (arm_zeroing_velocity_ < 0.0 &&
             estimated_arm() < average_arm - 2.0 * pulse_width) {
    arm_zeroing_velocity_ = kArmZeroingVelocity;
  }
  return arm_zeroing_velocity_;
}

void Fridge::RunIteration(const control_loops::FridgeQueue::Goal *goal,
                          const control_loops::FridgeQueue::Position *position,
                          control_loops::FridgeQueue::Output *output,
                          control_loops::FridgeQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    left_elevator_estimator_.Reset();
    right_elevator_estimator_.Reset();
    left_arm_estimator_.Reset();
    right_arm_estimator_.Reset();
    state_ = UNINITIALIZED;
  }

  // Get a reference to the constants struct since we use it so often in this
  // code.
  auto &values = constants::GetValues();

  // Bool to track if we should turn the motors on or not.
  bool disable = output == nullptr;
  double elevator_goal_velocity = 0.0;
  double arm_goal_velocity = 0.0;

  // Save the current position so it can be used easily in the class.
  current_position_ = *position;

  left_elevator_estimator_.UpdateEstimate(position->elevator.left);
  right_elevator_estimator_.UpdateEstimate(position->elevator.right);
  left_arm_estimator_.UpdateEstimate(position->arm.left);
  right_arm_estimator_.UpdateEstimate(position->arm.right);

  if (state_ != UNINITIALIZED) {
    Correct();
  }

  // Zeroing will work as follows:
  // At startup, record the offset of the two halves of the two subsystems.
  // Then, start moving the elevator towards the center until both halves are
  // zeroed.
  // Then, start moving the claw towards the center until both halves are
  // zeroed.
  // Then, done!

  // We'll then need code to do sanity checking on values.

  // Now, we need to figure out which way to go.

  switch (state_) {
    case UNINITIALIZED:
      LOG(DEBUG, "Uninitialized\n");
      // Startup.  Assume that we are at the origin everywhere.
      // This records the encoder offset between the two sides of the elevator.
      left_elevator_offset_ = -position->elevator.left.encoder;
      right_elevator_offset_ = -position->elevator.right.encoder;
      left_arm_offset_ = -position->arm.left.encoder;
      right_arm_offset_ = -position->arm.right.encoder;
      LOG(INFO, "Initializing arm offsets to %f, %f\n", left_arm_offset_,
          right_arm_offset_);
      LOG(INFO, "Initializing elevator offsets to %f, %f\n",
          left_elevator_offset_, right_elevator_offset_);
      Correct();
      state_ = INITIALIZING;
      disable = true;
      break;

    case INITIALIZING:
      LOG(DEBUG, "Waiting for accurate initial position.\n");
      disable = true;
      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();
      if (state_ != INITIALIZING) {
        // Set the goals to where we are now.
        elevator_goal_ = elevator();
        arm_goal_ = arm();
      }
      break;

    case ZEROING_ELEVATOR:
      LOG(DEBUG, "Zeroing elevator\n");

      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();
      if (left_elevator_estimator_.zeroed() &&
          right_elevator_estimator_.zeroed()) {
        SetElevatorOffset(left_elevator_estimator_.offset(),
                          right_elevator_estimator_.offset());
        LOG(DEBUG, "Zeroed the elevator!\n");
      } else if (!disable) {
        elevator_goal_velocity = elevator_zeroing_velocity();
        elevator_goal_ += elevator_goal_velocity * dt;
      }
      break;

    case ZEROING_ARM:
      LOG(DEBUG, "Zeroing the arm\n");

      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();
      if (left_arm_estimator_.zeroed() && right_arm_estimator_.zeroed()) {
        SetArmOffset(left_arm_estimator_.offset(),
                     right_arm_estimator_.offset());
        LOG(DEBUG, "Zeroed the arm!\n");
      } else if (!disable) {
        arm_goal_velocity = arm_zeroing_velocity();
        arm_goal_ += arm_goal_velocity * dt;
      }
      break;

    case RUNNING:
      LOG(DEBUG, "Running!\n");
      arm_goal_velocity = goal->angular_velocity;
      elevator_goal_velocity = goal->velocity;

      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();
      arm_goal_ = goal->angle;
      elevator_goal_ = goal->height;

      if (state_ != RUNNING && state_ != ESTOP) {
        state_ = UNINITIALIZED;
      }
      break;

    case ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // Commence death if either left/right tracking error gets too big. This
  // should run immediately after the SetArmOffset and SetElevatorOffset
  // functions to double-check that the hardware is in a sane state.
  if (::std::abs(left_arm() - right_arm()) >=
      values.max_allowed_left_right_arm_difference) {
    LOG(ERROR, "The arms are too far apart.  |%f - %f| > %f\n", left_arm(),
        right_arm(), values.max_allowed_left_right_arm_difference);

    // Indicate an ESTOP condition and stop the motors.
    state_ = ESTOP;
    disable = true;
  }

  if (::std::abs(left_elevator() - right_elevator()) >=
      values.max_allowed_left_right_elevator_difference) {
    LOG(ERROR, "The elevators are too far apart.  |%f - %f| > %f\n",
        left_elevator(), right_elevator(),
        values.max_allowed_left_right_elevator_difference);

    // Indicate an ESTOP condition and stop the motors.
    state_ = ESTOP;
    disable = true;
  }

  // Limit the goals so we can't exceed the hardware limits if we are RUNNING.
  if (state_ == RUNNING) {
    // Limit the arm goal to min/max allowable angles.
    if (arm_goal_ >= values.fridge.arm.upper_limit) {
      LOG(WARNING, "Arm goal above limit, %f > %f\n", arm_goal_,
          values.fridge.arm.upper_limit);
      arm_goal_ = values.fridge.arm.upper_limit;
    }
    if (arm_goal_ <= values.fridge.arm.lower_limit) {
      LOG(WARNING, "Arm goal below limit, %f < %f\n", arm_goal_,
          values.fridge.arm.lower_limit);
      arm_goal_ = values.fridge.arm.lower_limit;
    }

    // Limit the elevator goal to min/max allowable heights.
    if (elevator_goal_ >= values.fridge.elevator.upper_limit) {
      LOG(WARNING, "Elevator goal above limit, %f > %f\n", elevator_goal_,
          values.fridge.elevator.upper_limit);
      elevator_goal_ = values.fridge.elevator.upper_limit;
    }
    if (elevator_goal_ <= values.fridge.elevator.lower_limit) {
      LOG(WARNING, "Elevator goal below limit, %f < %f\n", elevator_goal_,
          values.fridge.elevator.lower_limit);
      elevator_goal_ = values.fridge.elevator.lower_limit;
    }
  }

  // Check the lower level hardware limit as well.
  if (state_ == RUNNING) {
    if (left_arm() >= values.fridge.arm.upper_hard_limit ||
        left_arm() <= values.fridge.arm.lower_hard_limit) {
      LOG(ERROR, "Left arm at %f out of bounds [%f, %f], ESTOPing\n",
          left_arm(), values.fridge.arm.lower_hard_limit,
          values.fridge.arm.upper_hard_limit);
      state_ = ESTOP;
    }

    if (right_arm() >= values.fridge.arm.upper_hard_limit ||
        right_arm() <= values.fridge.arm.lower_hard_limit) {
      LOG(ERROR, "Right arm at %f out of bounds [%f, %f], ESTOPing\n",
          right_arm(), values.fridge.arm.lower_hard_limit,
          values.fridge.arm.upper_hard_limit);
      state_ = ESTOP;
    }

    if (left_elevator() >= values.fridge.elevator.upper_hard_limit ||
        left_elevator() <= values.fridge.elevator.lower_hard_limit) {
      LOG(ERROR, "Left elevator at %f out of bounds [%f, %f], ESTOPing\n",
          left_elevator(), values.fridge.elevator.lower_hard_limit,
          values.fridge.elevator.upper_hard_limit);
      state_ = ESTOP;
    }

    if (right_elevator() >= values.fridge.elevator.upper_hard_limit ||
        right_elevator() <= values.fridge.elevator.lower_hard_limit) {
      LOG(ERROR, "Right elevator at %f out of bounds [%f, %f], ESTOPing\n",
          right_elevator(), values.fridge.elevator.lower_hard_limit,
          values.fridge.elevator.upper_hard_limit);
      state_ = ESTOP;
    }
  }

  // Set the goals.
  arm_loop_->mutable_R() << arm_goal_, arm_goal_velocity, 0.0, 0.0;
  elevator_loop_->mutable_R() << elevator_goal_, elevator_goal_velocity, 0.0,
      0.0;

  const double max_voltage = state_ == RUNNING ? 12.0 : kZeroingVoltage;
  arm_loop_->set_max_voltage(max_voltage);
  elevator_loop_->set_max_voltage(max_voltage);

  if (state_ == ESTOP) {
    disable = true;
  }
  arm_loop_->Update(disable);
  elevator_loop_->Update(disable);

  if (state_ == INITIALIZING || state_ == ZEROING_ELEVATOR ||
      state_ == ZEROING_ARM) {
    if (arm_loop_->U() != arm_loop_->U_uncapped()) {
      Eigen::Matrix<double, 2, 1> deltaR =
          arm_loop_->UnsaturateOutputGoalChange();

      // Move the average arm goal by the amount observed.
      LOG(WARNING, "Moving arm goal by %f to handle saturation\n",
          deltaR(0, 0));
      arm_goal_ += deltaR(0, 0);
    }

    if (elevator_loop_->U() != elevator_loop_->U_uncapped()) {
      Eigen::Matrix<double, 2, 1> deltaR =
          elevator_loop_->UnsaturateOutputGoalChange();

      // Move the average elevator goal by the amount observed.
      LOG(WARNING, "Moving elevator goal by %f to handle saturation\n",
          deltaR(0, 0));
      elevator_goal_ += deltaR(0, 0);
    }
  }

  if (output) {
    output->left_arm = arm_loop_->U(0, 0);
    output->right_arm = arm_loop_->U(1, 0);
    output->left_elevator = elevator_loop_->U(0, 0);
    output->right_elevator = elevator_loop_->U(1, 0);
    output->grabbers = goal->grabbers;
  }

  // TODO(austin): Populate these fully.
  status->zeroed = false;
  status->done = false;
  status->angle = arm_loop_->X_hat(0, 0);
  status->height = elevator_loop_->X_hat(0, 0);
  status->grabbers = goal->grabbers;
  status->estopped = (state_ == ESTOP);
  last_state_ = state_;
}

}  // namespace control_loops
}  // namespace frc971
