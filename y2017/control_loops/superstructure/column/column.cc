#include "y2017/control_loops/superstructure/column/column.h"

#include <array>
#include <chrono>
#include <memory>
#include <utility>

#include "Eigen/Dense"

#include "aos/common/commonmath.h"
#include "frc971/constants.h"
#include "frc971/control_loops/profiled_subsystem.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2017/control_loops/superstructure/column/column_integral_plant.h"
#include "y2017/control_loops/superstructure/column/stuck_column_integral_plant.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace column {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;
using ::frc971::zeroing::PulseIndexZeroingEstimator;

namespace {
constexpr double kTolerance = 10.0;
constexpr double kIndexerAcceleration = 50.0;
constexpr chrono::milliseconds kForwardTimeout{500};
constexpr chrono::milliseconds kReverseTimeout{1000};
constexpr chrono::milliseconds kReverseMinTimeout{500};
}  // namespace

constexpr double Column::kZeroingVoltage;
constexpr double Column::kOperatingVoltage;
constexpr double Column::kIntakeZeroingMinDistance;
constexpr double Column::kIntakeTolerance;
constexpr double Column::kStuckZeroingTrackingError;

ColumnProfiledSubsystem::ColumnProfiledSubsystem(
    ::std::unique_ptr<
        ::frc971::control_loops::SimpleCappedStateFeedbackLoop<6, 2, 2>>
        loop,
    const ::y2017::constants::Values::Column &zeroing_constants,
    const ::frc971::constants::Range &range, double default_velocity,
    double default_acceleration)
    : ProfiledSubsystem<6, 1, ColumnZeroingEstimator, 2, 2>(
          ::std::move(loop), {{zeroing_constants}}),
      stuck_indexer_detector_(new StateFeedbackLoop<6, 2, 2>(
          column::MakeStuckIntegralColumnLoop())),
      profile_(::aos::controls::kLoopFrequency),
      range_(range),
      default_velocity_(default_velocity),
      default_acceleration_(default_acceleration) {
  Y_.setZero();
  offset_.setZero();
  X_hat_current_.setZero();
  stuck_indexer_X_hat_current_.setZero();
  indexer_history_.fill(0);
  AdjustProfile(0.0, 0.0);
}

void ColumnProfiledSubsystem::AddOffset(double indexer_offset_delta,
                                        double turret_offset_delta) {
  UpdateOffset(offset_(0, 0) + indexer_offset_delta,
               offset_(1, 0) + turret_offset_delta);
}

void ColumnProfiledSubsystem::UpdateOffset(double indexer_offset,
                                           double turret_offset) {
  const double indexer_doffset = indexer_offset - offset_(0, 0);
  const double turret_doffset = turret_offset - offset_(1, 0);

  LOG(INFO, "Adjusting indexer offset from %f to %f\n", offset_(0, 0),
      indexer_offset);
  LOG(INFO, "Adjusting turret offset from %f to %f\n", offset_(1, 0),
      turret_offset);

  loop_->mutable_X_hat()(0, 0) += indexer_doffset;
  loop_->mutable_X_hat()(2, 0) += turret_doffset + indexer_doffset;

  stuck_indexer_detector_->mutable_X_hat()(0, 0) += indexer_doffset;
  stuck_indexer_detector_->mutable_X_hat()(2, 0) +=
      turret_doffset + indexer_doffset;
  Y_(0, 0) += indexer_doffset;
  Y_(1, 0) += turret_doffset;
  turret_last_position_ += turret_doffset + indexer_doffset;
  loop_->mutable_R(0, 0) += indexer_doffset;
  loop_->mutable_R(2, 0) += turret_doffset + indexer_doffset;

  profile_.MoveGoal(turret_doffset + indexer_doffset);
  offset_(0, 0) = indexer_offset;
  offset_(1, 0) = turret_offset;

  CapGoal("R", &loop_->mutable_R());
}

void ColumnProfiledSubsystem::Correct(const ColumnPosition &new_position) {
  estimators_[0].UpdateEstimate(new_position);

  if (estimators_[0].error()) {
    LOG(ERROR, "zeroing error\n");
    return;
  }

  if (!initialized_) {
    if (estimators_[0].offset_ready()) {
      UpdateOffset(estimators_[0].indexer_offset(),
                   estimators_[0].turret_offset());
      initialized_ = true;
    }
  }

  if (!zeroed(0) && estimators_[0].zeroed()) {
    UpdateOffset(estimators_[0].indexer_offset(),
                 estimators_[0].turret_offset());
    set_zeroed(0, true);
  }

  turret_last_position_ = turret_position();
  Y_ << new_position.indexer.position, new_position.turret.position;
  Y_ += offset_;
  loop_->Correct(Y_);

  indexer_history_[indexer_history_position_] = new_position.indexer.position;
  indexer_history_position_ = (indexer_history_position_ + 1) % kHistoryLength;

  indexer_dt_velocity_ =
      (new_position.indexer.position - indexer_last_position_) /
      chrono::duration_cast<chrono::duration<double>>(
          ::aos::controls::kLoopFrequency)
          .count();
  indexer_last_position_ = new_position.indexer.position;

  stuck_indexer_detector_->Correct(Y_);

  // Compute the oldest point in the history.
  const int indexer_oldest_history_position =
      ((indexer_history_position_ == 0) ? kHistoryLength
                                        : indexer_history_position_) -
      1;

  // Compute the distance moved over that time period.
  indexer_average_angular_velocity_ =
      (indexer_history_[indexer_oldest_history_position] -
       indexer_history_[indexer_history_position_]) /
      (chrono::duration_cast<chrono::duration<double>>(
           ::aos::controls::kLoopFrequency)
           .count() *
       static_cast<double>(kHistoryLength - 1));

  // Ready if average angular velocity is close to the goal.
  indexer_error_ = indexer_average_angular_velocity_ - unprofiled_goal_(1, 0);

  indexer_ready_ =
      std::abs(indexer_error_) < kTolerance && unprofiled_goal_(1, 0) > 0.1;

  // Pull state from the profiled subsystem.
  X_hat_current_ = controller().X_hat();
  stuck_indexer_X_hat_current_ = stuck_indexer_detector_->X_hat();
  indexer_position_error_ = X_hat_current_(0, 0) - Y(0, 0);
}

void ColumnProfiledSubsystem::CapGoal(const char *name,
                                      Eigen::Matrix<double, 6, 1> *goal) {
  // Limit the goal to min/max allowable positions.
  if (zeroed()) {
    if ((*goal)(2, 0) > range_.upper) {
      LOG(WARNING, "Goal %s above limit, %f > %f\n", name, (*goal)(2, 0),
          range_.upper);
      (*goal)(2, 0) = range_.upper;
    }
    if ((*goal)(2, 0) < range_.lower) {
      LOG(WARNING, "Goal %s below limit, %f < %f\n", name, (*goal)(2, 0),
          range_.lower);
      (*goal)(2, 0) = range_.lower;
    }
  } else {
    const double kMaxRange = range().upper_hard - range().lower_hard;

    // Limit the goal to min/max allowable positions much less agressively.
    // We don't know where the limits are, so we have to let the user move far
    // enough to find them (and the index pulse which might be right next to
    // one).
    // Upper - lower hard may be a bit generous, but we are moving slow.

    if ((*goal)(2, 0) > kMaxRange) {
      LOG(WARNING, "Goal %s above limit, %f > %f\n", name, (*goal)(2, 0),
          kMaxRange);
      (*goal)(2, 0) = kMaxRange;
    }
    if ((*goal)(2, 0) < -kMaxRange) {
      LOG(WARNING, "Goal %s below limit, %f < %f\n", name, (*goal)(2, 0),
          -kMaxRange);
      (*goal)(2, 0) = -kMaxRange;
    }
  }
}

void ColumnProfiledSubsystem::ForceGoal(double goal_velocity, double goal) {
  set_unprofiled_goal(goal_velocity, goal);
  loop_->mutable_R() = unprofiled_goal_;
  loop_->mutable_next_R() = loop_->R();

  const ::Eigen::Matrix<double, 6, 1> &R = loop_->R();
  profile_.MoveCurrentState(R.block<2, 1>(2, 0));
}

void ColumnProfiledSubsystem::set_unprofiled_goal(double goal_velocity,
                                                  double unprofiled_goal) {
  unprofiled_goal_(0, 0) = 0.0;
  unprofiled_goal_(1, 0) = goal_velocity;
  unprofiled_goal_(2, 0) = unprofiled_goal;
  unprofiled_goal_(3, 0) = 0.0;
  unprofiled_goal_(4, 0) = 0.0;
  unprofiled_goal_(5, 0) = 0.0;
  CapGoal("unprofiled R", &unprofiled_goal_);
}

void ColumnProfiledSubsystem::set_indexer_unprofiled_goal(
    double goal_velocity) {
  unprofiled_goal_(0, 0) = 0.0;
  unprofiled_goal_(1, 0) = goal_velocity;
  unprofiled_goal_(4, 0) = 0.0;
  CapGoal("unprofiled R", &unprofiled_goal_);
}

void ColumnProfiledSubsystem::set_turret_unprofiled_goal(
    double unprofiled_goal) {
  unprofiled_goal_(2, 0) = unprofiled_goal;
  unprofiled_goal_(3, 0) = 0.0;
  unprofiled_goal_(5, 0) = 0.0;
  CapGoal("unprofiled R", &unprofiled_goal_);
}

void ColumnProfiledSubsystem::Update(bool disable) {
  // TODO(austin): If we really need to reset, reset the profiles, etc.  That'll
  // be covered by the layer above when disabled though, so we can get away with
  // not doing it yet.
  if (should_reset_) {
    loop_->mutable_X_hat(0, 0) = Y_(0, 0);
    loop_->mutable_X_hat(1, 0) = 0.0;
    loop_->mutable_X_hat(2, 0) = Y_(0, 0) + Y_(1, 0);
    loop_->mutable_X_hat(3, 0) = 0.0;
    loop_->mutable_X_hat(4, 0) = 0.0;
    loop_->mutable_X_hat(5, 0) = 0.0;

    LOG(INFO, "Resetting\n");
    stuck_indexer_detector_->mutable_X_hat() = loop_->X_hat();
    should_reset_ = false;
    saturated_ = false;
  }

  if (!disable) {
    ::Eigen::Matrix<double, 2, 1> goal_state =
        profile_.Update(unprofiled_goal_(2, 0), unprofiled_goal_(3, 0));

    constexpr double kDt = chrono::duration_cast<chrono::duration<double>>(
                               ::aos::controls::kLoopFrequency)
                               .count();

    loop_->mutable_next_R(0, 0) = 0.0;
    // TODO(austin): This might not handle saturation right, but I'm not sure I
    // really care.
    loop_->mutable_next_R(1, 0) = ::aos::Clip(
        unprofiled_goal_(1, 0), loop_->R(1, 0) - kIndexerAcceleration * kDt,
        loop_->R(1, 0) + kIndexerAcceleration * kDt);
    loop_->mutable_next_R(2, 0) = goal_state(0, 0);
    loop_->mutable_next_R(3, 0) = goal_state(1, 0);
    loop_->mutable_next_R(4, 0) = 0.0;
    loop_->mutable_next_R(5, 0) = 0.0;
    CapGoal("next R", &loop_->mutable_next_R());
  }

  // If the indexer goal velocity is low, switch to the indexer controller which
  // won't fight to keep it moving at 0.
  if (::std::abs(unprofiled_goal_(1, 0)) < 0.1) {
    loop_->set_index(1);
  } else {
    loop_->set_index(0);
  }
  loop_->Update(disable);

  if (!disable && loop_->U(1, 0) != loop_->U_uncapped(1, 0)) {
    const ::Eigen::Matrix<double, 6, 1> &R = loop_->R();
    profile_.MoveCurrentState(R.block<2, 1>(2, 0));
    saturated_ = true;
  } else {
    saturated_ = false;
  }

  // Run the KF predict step.
  stuck_indexer_detector_->UpdateObserver(loop_->U(),
                                          ::aos::controls::kLoopFrequency);
}

bool ColumnProfiledSubsystem::CheckHardLimits() {
  // Returns whether hard limits have been exceeded.

  if (turret_position() > range_.upper_hard || turret_position() < range_.lower_hard) {
    LOG(ERROR,
        "ColumnProfiledSubsystem at %f out of bounds [%f, %f], ESTOPing\n",
        turret_position(), range_.lower_hard, range_.upper_hard);
    return true;
  }

  return false;
}

void ColumnProfiledSubsystem::AdjustProfile(
    const ::frc971::ProfileParameters &profile_parameters) {
  AdjustProfile(profile_parameters.max_velocity,
                profile_parameters.max_acceleration);
}

void ColumnProfiledSubsystem::AdjustProfile(double max_angular_velocity,
                                            double max_angular_acceleration) {
  profile_.set_maximum_velocity(
      ::frc971::control_loops::internal::UseUnlessZero(max_angular_velocity,
                                                       default_velocity_));
  profile_.set_maximum_acceleration(
      ::frc971::control_loops::internal::UseUnlessZero(max_angular_acceleration,
                                                       default_acceleration_));
}

double ColumnProfiledSubsystem::IndexerStuckVoltage() const {
  // Compute the voltage from the control loop, excluding the voltage error
  // term.
  const double uncapped_applied_voltage =
      uncapped_indexer_voltage() + X_hat(4, 0);
  if (uncapped_applied_voltage < 0) {
    return +stuck_indexer_X_hat_current_(4, 0);
  } else {
    return -stuck_indexer_X_hat_current_(4, 0);
  }
}
bool ColumnProfiledSubsystem::IsIndexerStuck() const {
  return IndexerStuckVoltage() > 4.0;
}

void ColumnProfiledSubsystem::PartialIndexerReset() {
  mutable_X_hat(4, 0) = 0.0;
  stuck_indexer_detector_->mutable_X_hat(4, 0) = 0.0;
  // Screw it, we are stuck.  Reset the current goal to the current velocity so
  // we start slewing faster to reverse if we have stopped.
  loop_->mutable_R(1, 0) = X_hat(1, 0);
  loop_->mutable_next_R(1, 0) = X_hat(1, 0);
}

void ColumnProfiledSubsystem::PartialTurretReset() {
  mutable_X_hat(5, 0) = 0.0;
  stuck_indexer_detector_->mutable_X_hat(5, 0) = 0.0;
}

void ColumnProfiledSubsystem::PopulateIndexerStatus(IndexerStatus *status) {
  status->avg_angular_velocity = indexer_average_angular_velocity_;

  status->angular_velocity = X_hat_current_(1, 0);
  status->ready = indexer_ready_;

  status->voltage_error = X_hat_current_(4, 0);
  status->stuck_voltage_error = stuck_indexer_X_hat_current_(4, 0);
  status->position_error = indexer_position_error_;
  status->instantaneous_velocity = indexer_dt_velocity_;

  status->stuck = IsIndexerStuck();

  status->stuck_voltage = IndexerStuckVoltage();
}

Column::Column()
    : profiled_subsystem_(
          ::std::unique_ptr<
              ::frc971::control_loops::SimpleCappedStateFeedbackLoop<6, 2, 2>>(
              new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<
                  6, 2, 2>(MakeIntegralColumnLoop())),
          constants::GetValues().column, constants::Values::kTurretRange, 7.0,
          50.0),
      vision_error_(constants::GetValues().vision_error) {}

void Column::Reset() {
  state_ = State::UNINITIALIZED;
  indexer_state_ = IndexerState::RUNNING;
  profiled_subsystem_.Reset();
  // intake will automatically clear the minimum position on reset, so we don't
  // need to do it here.
  freeze_ = false;
}

void Column::Iterate(const control_loops::IndexerGoal *unsafe_indexer_goal,
                     const control_loops::TurretGoal *unsafe_turret_goal,
                     const ColumnPosition *position,
                     const vision::VisionStatus *vision_status,
                     double *indexer_output, double *turret_output,
                     IndexerStatus *indexer_status,
                     TurretProfiledSubsystemStatus *turret_status,
                     intake::Intake *intake) {
  bool disable = turret_output == nullptr || indexer_output == nullptr;
  profiled_subsystem_.Correct(*position);

  vision_time_adjuster_.Tick(::aos::monotonic_clock::now(),
                             profiled_subsystem_.X_hat(2, 0), vision_status);

  switch (state_) {
    case State::UNINITIALIZED:
      // Wait in the uninitialized state until the turret is initialized.
      // Set the goals to where we are now so when we start back up, we don't
      // jump.
      profiled_subsystem_.ForceGoal(0.0, profiled_subsystem_.turret_position());
      state_ = State::ZEROING_UNINITIALIZED;

      // Fall through so we can start the zeroing process immediately.

    case State::ZEROING_UNINITIALIZED:
      // Set up the profile to be the zeroing profile.
      profiled_subsystem_.AdjustProfile(0.50, 3);

      // Force the intake out.
      intake->set_min_position(kIntakeZeroingMinDistance);

      if (disable) {
        // If we are disabled, we want to reset the turret to stay where it
        // currently is.
        profiled_subsystem_.ForceGoal(0.0,
                                      profiled_subsystem_.turret_position());
      } else if (profiled_subsystem_.initialized()) {
        // If we are initialized, there's no value in continuing to move so stop
        // and wait on the intake.
        profiled_subsystem_.set_indexer_unprofiled_goal(0.0);
      } else {
        // Spin slowly backwards.
        profiled_subsystem_.set_indexer_unprofiled_goal(2.0);
      }

      // See if we are zeroed or initialized and far enough out and execute the
      // proper state transition.
      if (profiled_subsystem_.zeroed()) {
        intake->clear_min_position();
        state_ = State::RUNNING;
      } else if (profiled_subsystem_.initialized() &&
                 intake->position() >
                     kIntakeZeroingMinDistance - kIntakeTolerance) {
        if (profiled_subsystem_.turret_position() > 0) {
          // We need to move in the negative direction.
          state_ = State::ZEROING_NEGATIVE;
        } else {
          // We need to move in the positive direction.
          state_ = State::ZEROING_POSITIVE;
        }
      }
      break;

    case State::ZEROING_POSITIVE:
      // We are now going to be moving in the positive direction towards 0.  If
      // we get close enough, we'll zero.
      profiled_subsystem_.set_unprofiled_goal(0.0, 0.0);
      intake->set_min_position(kIntakeZeroingMinDistance);

      if (profiled_subsystem_.zeroed()) {
        intake->clear_min_position();
        state_ = State::RUNNING;
      } else if (disable) {
        // We are disabled, so pick back up from the current position.
        profiled_subsystem_.ForceGoal(0.0,
                                      profiled_subsystem_.turret_position());
      } else if (profiled_subsystem_.turret_position() <
                     profiled_subsystem_.goal(2, 0) -
                         kStuckZeroingTrackingError ||
                 profiled_subsystem_.saturated()) {
        LOG(INFO,
            "Turret stuck going positive, switching directions.  At %f, goal "
            "%f\n",
            profiled_subsystem_.turret_position(),
            profiled_subsystem_.goal(2, 0));
        // The turret got too far behind.  Declare it stuck and reverse.
        profiled_subsystem_.AddOffset(0.0, 2.0 * M_PI);
        profiled_subsystem_.set_unprofiled_goal(0.0, 0.0);
        profiled_subsystem_.ForceGoal(0.0,
                                      profiled_subsystem_.turret_position());
        profiled_subsystem_.PartialTurretReset();
        profiled_subsystem_.PartialIndexerReset();
        state_ = State::ZEROING_NEGATIVE;
      }
      break;

    case State::ZEROING_NEGATIVE:
      // We are now going to be moving in the negative direction towards 0.  If
      // we get close enough, we'll zero.
      profiled_subsystem_.set_unprofiled_goal(0.0, 0.0);
      intake->set_min_position(kIntakeZeroingMinDistance);

      if (profiled_subsystem_.zeroed()) {
        intake->clear_min_position();
        state_ = State::RUNNING;
      } else if (disable) {
        // We are disabled, so pick back up from the current position.
        profiled_subsystem_.ForceGoal(0.0,
                                      profiled_subsystem_.turret_position());
      } else if (profiled_subsystem_.turret_position() >
                     profiled_subsystem_.goal(2, 0) +
                         kStuckZeroingTrackingError ||
                 profiled_subsystem_.saturated()) {
        // The turret got too far behind.  Declare it stuck and reverse.
        LOG(INFO,
            "Turret stuck going negative, switching directions.  At %f, goal "
            "%f\n",
            profiled_subsystem_.turret_position(),
            profiled_subsystem_.goal(2, 0));
        profiled_subsystem_.AddOffset(0.0, -2.0 * M_PI);
        profiled_subsystem_.set_unprofiled_goal(0.0, 0.0);
        profiled_subsystem_.ForceGoal(0.0,
                                      profiled_subsystem_.turret_position());
        profiled_subsystem_.PartialTurretReset();
        profiled_subsystem_.PartialIndexerReset();
        state_ = State::ZEROING_POSITIVE;
      }
      break;

    case State::RUNNING: {
      double starting_goal_angle = profiled_subsystem_.goal(2, 0);
      if (disable) {
        // Reset the profile to the current position so it starts from here when
        // we get re-enabled.
        profiled_subsystem_.ForceGoal(0.0,
                                      profiled_subsystem_.turret_position());
      }

      if (unsafe_turret_goal && unsafe_indexer_goal) {
        profiled_subsystem_.AdjustProfile(unsafe_turret_goal->profile_params);
        profiled_subsystem_.set_unprofiled_goal(
            unsafe_indexer_goal->angular_velocity, unsafe_turret_goal->angle);

        if (unsafe_turret_goal->track) {
          if (vision_time_adjuster_.valid()) {
            LOG(INFO, "Vision aligning to %f\n", vision_time_adjuster_.goal());
            profiled_subsystem_.set_turret_unprofiled_goal(
                vision_time_adjuster_.goal() + vision_error_);
          }
        } else {
          vision_time_adjuster_.ResetTime();
        }

        if (freeze_) {
          profiled_subsystem_.ForceGoal(unsafe_indexer_goal->angular_velocity,
                                        starting_goal_angle);
        }
      }

      // ESTOP if we hit the hard limits.
      if (profiled_subsystem_.CheckHardLimits() ||
          profiled_subsystem_.error()) {
        state_ = State::ESTOP;
      }
    } break;

    case State::ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // Start indexing at the suggested velocity.
  // If a "stuck" event is detected, reverse.  Stay reversed until either
  // unstuck, or 0.5 seconds have elapsed.
  // Then, start going forwards.  Don't detect stuck for 0.5 seconds.

  monotonic_clock::time_point monotonic_now = monotonic_clock::now();
  switch (indexer_state_) {
    case IndexerState::RUNNING:
      // The velocity goal is already set above in this case, so leave it
      // alone.

      // If we are stuck and weren't just reversing, try reversing to unstick
      // us.  We don't want to chatter back and forth too fast if reversing
      // isn't working.
      if (profiled_subsystem_.IsIndexerStuck() &&
          monotonic_now > kForwardTimeout + last_transition_time_) {
        indexer_state_ = IndexerState::REVERSING;
        last_transition_time_ = monotonic_now;
        profiled_subsystem_.PartialIndexerReset();
        LOG(INFO, "Partial indexer reset while going forwards\n");
        LOG(INFO, "Indexer RUNNING -> REVERSING\n");
      }
      break;
    case IndexerState::REVERSING:
      // "Reverse" "slowly".
      profiled_subsystem_.set_indexer_unprofiled_goal(
          -5.0 * ::aos::sign(profiled_subsystem_.unprofiled_goal(1, 0)));

      // If we've timed out or are no longer stuck, try running again.
      if ((!profiled_subsystem_.IsIndexerStuck() &&
           monotonic_now > last_transition_time_ + kReverseMinTimeout) ||
          monotonic_now > kReverseTimeout + last_transition_time_) {
        indexer_state_ = IndexerState::RUNNING;
        LOG(INFO, "Indexer REVERSING -> RUNNING, stuck %d\n",
            profiled_subsystem_.IsIndexerStuck());

        // Only reset if we got stuck going this way too.
        if (monotonic_now > kReverseTimeout + last_transition_time_) {
          LOG(INFO, "Partial indexer reset while reversing\n");
          profiled_subsystem_.PartialIndexerReset();
        }
        last_transition_time_ = monotonic_now;
      }
      break;
  }

  // Set the voltage limits.
  const double max_voltage =
      (state_ == State::RUNNING) ? kOperatingVoltage : kZeroingVoltage;

  profiled_subsystem_.set_max_voltage({{max_voltage, max_voltage}});

  // Calculate the loops for a cycle.
  profiled_subsystem_.Update(disable);

  // Write out all the voltages.
  if (indexer_output) {
    *indexer_output = profiled_subsystem_.indexer_voltage();
  }
  if (turret_output) {
    *turret_output = profiled_subsystem_.turret_voltage();
  }

  // Save debug/internal state.
  // TODO(austin): Save more.
  turret_status->zeroed = profiled_subsystem_.zeroed();
  profiled_subsystem_.PopulateTurretStatus(turret_status);
  turret_status->estopped = (state_ == State::ESTOP);
  turret_status->state = static_cast<int32_t>(state_);
  turret_status->turret_encoder_angle = profiled_subsystem_.turret_position();
  if (vision_time_adjuster_.valid()) {
    turret_status->vision_angle = vision_time_adjuster_.goal();
    turret_status->raw_vision_angle =
        vision_time_adjuster_.most_recent_vision_reading();
    turret_status->vision_tracking = true;
  } else {
    turret_status->vision_angle = ::std::numeric_limits<double>::quiet_NaN();
    turret_status->vision_tracking = false;
  }

  profiled_subsystem_.PopulateIndexerStatus(indexer_status);
  indexer_status->state = static_cast<int32_t>(indexer_state_);
}

}  // namespace column
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
