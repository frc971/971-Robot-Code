#include "y2015/control_loops/fridge/fridge.h"

#include <cmath>

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "y2015/control_loops/fridge/elevator_motor_plant.h"
#include "y2015/control_loops/fridge/integral_arm_plant.h"
#include "frc971/control_loops/voltage_cap/voltage_cap.h"
#include "frc971/zeroing/zeroing.h"

#include "y2015/constants.h"

namespace y2015 {
namespace control_loops {
namespace fridge {

namespace chrono = ::std::chrono;

namespace {
constexpr double kZeroingVoltage = 4.0;
constexpr double kElevatorZeroingVelocity = 0.10;
// What speed we move to our safe height at.
constexpr double kElevatorSafeHeightVelocity = 0.3;
constexpr double kArmZeroingVelocity = 0.20;
}  // namespace

template <int S>
void CappedStateFeedbackLoop<S>::CapU() {
  ::frc971::control_loops::VoltageCap(max_voltage_, this->U(0, 0),
                                      this->U(1, 0), &this->mutable_U(0, 0),
                                      &this->mutable_U(1, 0));
}

template <int S>
Eigen::Matrix<double, 2, 1>
CappedStateFeedbackLoop<S>::UnsaturateOutputGoalChange() {
  // Compute the K matrix used to compensate for position errors.
  Eigen::Matrix<double, 2, 2> Kp;
  Kp.setZero();
  Kp.col(0) = this->controller().K().col(0);
  Kp.col(1) = this->controller().K().col(2);

  Eigen::Matrix<double, 2, 2> Kp_inv = Kp.inverse();

  // Compute how much we need to change R in order to achieve the change in U
  // that was observed.
  Eigen::Matrix<double, 2, 1> deltaR =
      -Kp_inv * (this->U_uncapped() - this->U());
  return deltaR;
}

Fridge::Fridge(FridgeQueue *fridge)
    : aos::controls::ControlLoop<FridgeQueue>(fridge),
      arm_loop_(new CappedStateFeedbackLoop<5>(StateFeedbackLoop<5, 2, 2>(
          MakeIntegralArmLoop()))),
      elevator_loop_(new CappedStateFeedbackLoop<4>(
          StateFeedbackLoop<4, 2, 2>(MakeElevatorLoop()))),
      left_arm_estimator_(constants::GetValues().fridge.left_arm_zeroing),
      right_arm_estimator_(constants::GetValues().fridge.right_arm_zeroing),
      left_elevator_estimator_(constants::GetValues().fridge.left_elev_zeroing),
      right_elevator_estimator_(
          constants::GetValues().fridge.right_elev_zeroing),
      last_profiling_type_(ProfilingType::ANGLE_HEIGHT_PROFILING),
      kinematics_(constants::GetValues().fridge.arm_length,
                  constants::GetValues().fridge.elevator.upper_limit,
                  constants::GetValues().fridge.elevator.lower_limit,
                  constants::GetValues().fridge.arm.upper_limit,
                  constants::GetValues().fridge.arm.lower_limit),
      arm_profile_(::aos::controls::kLoopFrequency),
      elevator_profile_(::aos::controls::kLoopFrequency),
      x_profile_(::aos::controls::kLoopFrequency),
      y_profile_(::aos::controls::kLoopFrequency) {}

void Fridge::UpdateZeroingState() {
  if (left_elevator_estimator_.offset_ratio_ready() < 1.0 ||
      right_elevator_estimator_.offset_ratio_ready() < 1.0 ||
      left_arm_estimator_.offset_ratio_ready() < 1.0 ||
      right_arm_estimator_.offset_ratio_ready() < 1.0) {
    state_ = INITIALIZING;
  } else if (!left_elevator_estimator_.zeroed() ||
             !right_elevator_estimator_.zeroed()) {
    state_ = ZEROING_ELEVATOR;
  } else if (!left_arm_estimator_.zeroed() || !right_arm_estimator_.zeroed()) {
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
  return current_position_.arm.left.encoder + left_arm_estimator_.offset();
}
double Fridge::estimated_right_arm() {
  return current_position_.arm.right.encoder + right_arm_estimator_.offset();
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
             estimated_elevator() > average_elevator + 1.1 * pulse_width) {
    elevator_zeroing_velocity_ = -kElevatorZeroingVelocity;
  } else if (elevator_zeroing_velocity_ < 0 &&
             estimated_elevator() < average_elevator - 1.1 * pulse_width) {
    elevator_zeroing_velocity_ = kElevatorZeroingVelocity;
  }
  return elevator_zeroing_velocity_;
}

double Fridge::UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
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
             estimated_arm() > average_arm + 1.1 * pulse_width) {
    arm_zeroing_velocity_ = -kArmZeroingVelocity;
  } else if (arm_zeroing_velocity_ < 0.0 && estimated_arm() < average_arm) {
    arm_zeroing_velocity_ = kArmZeroingVelocity;
  }
  return arm_zeroing_velocity_;
}

void Fridge::RunIteration(const FridgeQueue::Goal *unsafe_goal,
                          const FridgeQueue::Position *position,
                          FridgeQueue::Output *output,
                          FridgeQueue::Status *status) {
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
  const auto &values = constants::GetValues();

  // Bool to track if we should turn the motors on or not.
  bool disable = output == nullptr;

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
      elevator_loop_->mutable_X_hat().setZero();
      arm_loop_->mutable_X_hat().setZero();
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

        if (elevator() < values.fridge.arm_zeroing_height &&
            state_ != INITIALIZING) {
          // Move the elevator to a safe height before we start zeroing the arm,
          // so that we don't crash anything.
          LOG(DEBUG, "Moving elevator to safe height.\n");
          if (elevator_goal_ < values.fridge.arm_zeroing_height) {
            elevator_goal_ += kElevatorSafeHeightVelocity *
                              chrono::duration_cast<chrono::duration<double>>(
                                  ::aos::controls::kLoopFrequency).count();
            elevator_goal_velocity_ = kElevatorSafeHeightVelocity;
            state_ = ZEROING_ELEVATOR;
          } else {
            // We want it stopped at whatever height it's currently set to.
            elevator_goal_velocity_ = 0;
          }
        }
      } else if (!disable) {
        elevator_goal_velocity_ = elevator_zeroing_velocity();
        elevator_goal_ += elevator_goal_velocity_ *
                          chrono::duration_cast<chrono::duration<double>>(
                              ::aos::controls::kLoopFrequency).count();
      }

      // Bypass motion profiles while we are zeroing.
      // This is also an important step right after the elevator is zeroed and
      // we reach into the elevator's state matrix and change it based on the
      // newly-obtained offset.
      {
        Eigen::Matrix<double, 2, 1> current;
        current.setZero();
        current << elevator_goal_, elevator_goal_velocity_;
        elevator_profile_.MoveCurrentState(current);
      }
      break;

    case ZEROING_ARM:
      LOG(DEBUG, "Zeroing the arm\n");

      if (elevator() < values.fridge.arm_zeroing_height - 0.10 ||
          elevator_goal_ < values.fridge.arm_zeroing_height) {
        LOG(INFO,
            "Going back to ZEROING_ELEVATOR until it gets high enough to "
            "safely zero the arm\n");
        state_ = ZEROING_ELEVATOR;
        break;
      }

      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();
      if (left_arm_estimator_.zeroed() && right_arm_estimator_.zeroed()) {
        SetArmOffset(left_arm_estimator_.offset(),
                     right_arm_estimator_.offset());
        LOG(DEBUG, "Zeroed the arm!\n");
      } else if (!disable) {
        arm_goal_velocity_ = arm_zeroing_velocity();
        arm_goal_ += arm_goal_velocity_ *
                     chrono::duration_cast<chrono::duration<double>>(
                         ::aos::controls::kLoopFrequency).count();
      }

      // Bypass motion profiles while we are zeroing.
      // This is also an important step right after the arm is zeroed and
      // we reach into the arm's state matrix and change it based on the
      // newly-obtained offset.
      {
        Eigen::Matrix<double, 2, 1> current;
        current.setZero();
        current << arm_goal_, arm_goal_velocity_;
        arm_profile_.MoveCurrentState(current);
      }
      break;

    case RUNNING:
      LOG(DEBUG, "Running!\n");
      if (unsafe_goal) {
        // Handle the case where we switch between the types of profiling.
        ProfilingType new_profiling_type =
            static_cast<ProfilingType>(unsafe_goal->profiling_type);

        if (last_profiling_type_ != new_profiling_type) {
          // Reset the height/angle profiles.
          Eigen::Matrix<double, 2, 1> current;
          current.setZero();
          current << arm_goal_, arm_goal_velocity_;
          arm_profile_.MoveCurrentState(current);
          current << elevator_goal_, elevator_goal_velocity_;
          elevator_profile_.MoveCurrentState(current);

          // Reset the x/y profiles.
          aos::util::ElevatorArmKinematics::KinematicResult x_y_result;
          kinematics_.ForwardKinematic(elevator_goal_, arm_goal_,
                                       elevator_goal_velocity_,
                                       arm_goal_velocity_, &x_y_result);
          current << x_y_result.fridge_x, x_y_result.fridge_x_velocity;
          x_profile_.MoveCurrentState(current);
          current << x_y_result.fridge_h, x_y_result.fridge_h_velocity;
          y_profile_.MoveCurrentState(current);

          last_profiling_type_ = new_profiling_type;
        }

        if (new_profiling_type == ProfilingType::ANGLE_HEIGHT_PROFILING) {
          // Pick a set of sane arm defaults if none are specified.
          arm_profile_.set_maximum_velocity(
              UseUnlessZero(unsafe_goal->max_angular_velocity, 1.0));
          arm_profile_.set_maximum_acceleration(
              UseUnlessZero(unsafe_goal->max_angular_acceleration, 3.0));
          elevator_profile_.set_maximum_velocity(
              UseUnlessZero(unsafe_goal->max_velocity, 0.50));
          elevator_profile_.set_maximum_acceleration(
              UseUnlessZero(unsafe_goal->max_acceleration, 2.0));

          // Use the profiles to limit the arm's movements.
          const double unfiltered_arm_goal = ::std::max(
              ::std::min(unsafe_goal->angle, values.fridge.arm.upper_limit),
              values.fridge.arm.lower_limit);
          ::Eigen::Matrix<double, 2, 1> arm_goal_state = arm_profile_.Update(
              unfiltered_arm_goal, unsafe_goal->angular_velocity);
          arm_goal_ = arm_goal_state(0, 0);
          arm_goal_velocity_ = arm_goal_state(1, 0);

          // Use the profiles to limit the elevator's movements.
          const double unfiltered_elevator_goal =
              ::std::max(::std::min(unsafe_goal->height,
                                    values.fridge.elevator.upper_limit),
                         values.fridge.elevator.lower_limit);
          ::Eigen::Matrix<double, 2, 1> elevator_goal_state =
              elevator_profile_.Update(unfiltered_elevator_goal,
                                       unsafe_goal->velocity);
          elevator_goal_ = elevator_goal_state(0, 0);
          elevator_goal_velocity_ = elevator_goal_state(1, 0);
        } else if (new_profiling_type == ProfilingType::X_Y_PROFILING) {
          // Use x/y profiling
          aos::util::ElevatorArmKinematics::KinematicResult kinematic_result;

          x_profile_.set_maximum_velocity(
              UseUnlessZero(unsafe_goal->max_x_velocity, 0.5));
          x_profile_.set_maximum_acceleration(
              UseUnlessZero(unsafe_goal->max_x_acceleration, 2.0));
          y_profile_.set_maximum_velocity(
              UseUnlessZero(unsafe_goal->max_y_velocity, 0.50));
          y_profile_.set_maximum_acceleration(
              UseUnlessZero(unsafe_goal->max_y_acceleration, 2.0));

          // Limit the goals before we update the profiles.
          kinematics_.InverseKinematic(
              unsafe_goal->x, unsafe_goal->y, unsafe_goal->x_velocity,
              unsafe_goal->y_velocity, &kinematic_result);

          // Use the profiles to limit the x movements.
          ::Eigen::Matrix<double, 2, 1> x_goal_state = x_profile_.Update(
              kinematic_result.fridge_x, kinematic_result.fridge_x_velocity);

          // Use the profiles to limit the y movements.
          ::Eigen::Matrix<double, 2, 1> y_goal_state = y_profile_.Update(
              kinematic_result.fridge_h, kinematic_result.fridge_h_velocity);

          // Convert x/y goal states into arm/elevator goals.
          // The inverse kinematics functions automatically perform range
          // checking and adjust the results so that they're always valid.
          kinematics_.InverseKinematic(x_goal_state(0, 0), y_goal_state(0, 0),
                                       x_goal_state(1, 0), y_goal_state(1, 0),
                                       &kinematic_result);

          // Store the appropriate inverse kinematic results in the
          // arm/elevator goals.
          arm_goal_ = kinematic_result.arm_angle;
          arm_goal_velocity_ = kinematic_result.arm_velocity;

          elevator_goal_ = kinematic_result.elevator_height;
          elevator_goal_velocity_ = kinematic_result.arm_velocity;
        } else {
          LOG(ERROR, "Unknown profiling_type: %d\n",
              unsafe_goal->profiling_type);
        }
      }

      // Update state_ to accurately represent the state of the zeroing
      // estimators.
      UpdateZeroingState();

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
    if (output) {
      state_ = ESTOP;
    }
    disable = true;
  }

  if (::std::abs(left_elevator() - right_elevator()) >=
      values.max_allowed_left_right_elevator_difference) {
    LOG(ERROR, "The elevators are too far apart.  |%f - %f| > %f\n",
        left_elevator(), right_elevator(),
        values.max_allowed_left_right_elevator_difference);

    // Indicate an ESTOP condition and stop the motors.
    if (output) {
      state_ = ESTOP;
    }
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
      if (output) {
        state_ = ESTOP;
      }
    }

    if (right_arm() >= values.fridge.arm.upper_hard_limit ||
        right_arm() <= values.fridge.arm.lower_hard_limit) {
      LOG(ERROR, "Right arm at %f out of bounds [%f, %f], ESTOPing\n",
          right_arm(), values.fridge.arm.lower_hard_limit,
          values.fridge.arm.upper_hard_limit);
      if (output) {
        state_ = ESTOP;
      }
    }

    if (left_elevator() >= values.fridge.elevator.upper_hard_limit) {
      LOG(ERROR, "Left elevator at %f out of bounds [%f, %f], ESTOPing\n",
          left_elevator(), values.fridge.elevator.lower_hard_limit,
          values.fridge.elevator.upper_hard_limit);
      if (output) {
        state_ = ESTOP;
      }
    }

    if (right_elevator() >= values.fridge.elevator.upper_hard_limit) {
      LOG(ERROR, "Right elevator at %f out of bounds [%f, %f], ESTOPing\n",
          right_elevator(), values.fridge.elevator.lower_hard_limit,
          values.fridge.elevator.upper_hard_limit);
      if (output) {
        state_ = ESTOP;
      }
    }
  }

  // Set the goals.
  arm_loop_->mutable_R() << arm_goal_, arm_goal_velocity_, 0.0, 0.0, 0.0;
  elevator_loop_->mutable_R() << elevator_goal_, elevator_goal_velocity_, 0.0,
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
    if (unsafe_goal) {
      output->grabbers = unsafe_goal->grabbers;
    } else {
      output->grabbers.top_front = false;
      output->grabbers.top_back = false;
      output->grabbers.bottom_front = false;
      output->grabbers.bottom_back = false;
    }
  }

  // TODO(austin): Populate these fully.
  status->zeroed = state_ == RUNNING;

  status->angle = arm_loop_->X_hat(0, 0);
  status->angular_velocity = arm_loop_->X_hat(1, 0);
  status->height = elevator_loop_->X_hat(0, 0);
  status->velocity = elevator_loop_->X_hat(1, 0);

  status->goal_angle = arm_goal_;
  status->goal_angular_velocity = arm_goal_velocity_;
  status->goal_height = elevator_goal_;
  status->goal_velocity = elevator_goal_velocity_;

  // Populate the same status, but in X/Y co-ordinates.
  aos::util::ElevatorArmKinematics::KinematicResult x_y_status;
  kinematics_.ForwardKinematic(status->height, status->angle,
                               status->velocity, status->angular_velocity,
                               &x_y_status);
  status->x = x_y_status.fridge_x;
  status->y = x_y_status.fridge_h;
  status->x_velocity = x_y_status.fridge_x_velocity;
  status->y_velocity = x_y_status.fridge_h_velocity;

  kinematics_.ForwardKinematic(status->goal_height, status->goal_angle,
                               status->goal_velocity, status->goal_angular_velocity,
                               &x_y_status);
  status->goal_x = x_y_status.fridge_x;
  status->goal_y = x_y_status.fridge_h;
  status->goal_x_velocity = x_y_status.fridge_x_velocity;
  status->goal_y_velocity = x_y_status.fridge_h_velocity;

  if (unsafe_goal) {
    status->grabbers = unsafe_goal->grabbers;
  } else {
    status->grabbers.top_front = false;
    status->grabbers.top_back = false;
    status->grabbers.bottom_front = false;
    status->grabbers.bottom_back = false;
  }
  status->left_arm_state = left_arm_estimator_.GetEstimatorState();
  status->right_arm_state = right_arm_estimator_.GetEstimatorState();
  status->left_elevator_state = left_elevator_estimator_.GetEstimatorState();
  status->right_elevator_state = right_elevator_estimator_.GetEstimatorState();
  status->estopped = (state_ == ESTOP);
  status->state = state_;
  last_state_ = state_;
}

}  // namespace fridge
}  // namespace control_loops
}  // namespace y2015
