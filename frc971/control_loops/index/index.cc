#include "frc971/control_loops/index/index.h"

#include <stdio.h>

#include <algorithm>

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/index/index_motor_plant.h"

using ::aos::time::Time;

namespace frc971 {
namespace control_loops {

double IndexMotor::Frisbee::ObserveNoTopDiscSensor(
    double index_position, double index_velocity) {
  // The absolute disc position in meters.
  double disc_position = IndexMotor::ConvertIndexToDiscPosition(
      index_position - index_start_position_) + IndexMotor::kIndexStartPosition;
  if (IndexMotor::kTopDiscDetectStart <= disc_position &&
      disc_position <= IndexMotor::kTopDiscDetectStop) {
    // Whoops, this shouldn't be happening.
    // Move the disc off the way that makes most sense.
    double distance_to_above = IndexMotor::ConvertDiscPositionToIndex(
        ::std::abs(disc_position - IndexMotor::kTopDiscDetectStop));
    double distance_to_below = IndexMotor::ConvertDiscPositionToIndex(
        ::std::abs(disc_position - IndexMotor::kTopDiscDetectStart));
    if (::std::abs(index_velocity) < 100) {
      if (distance_to_above < distance_to_below) {
        printf("Moving disc to top slow.\n");
        // Move it up.
        index_start_position_ -= distance_to_above;
        return -distance_to_above;
      } else {
        printf("Moving disc to bottom slow.\n");
        index_start_position_ += distance_to_below;
        return distance_to_below;
      }
    } else {
      if (index_velocity > 0) {
        // Now going up.  If we didn't see it before, and we don't see it
        // now but it should be in view, it must still be below.  If it were
        // above, it would be going further away from us.
        printf("Moving fast up, shifting disc up\n");
        index_start_position_ += distance_to_below;
        return distance_to_below;
      } else {
        printf("Moving fast down, shifting disc down\n");
        index_start_position_ -= distance_to_above;
        return -distance_to_above;
      }
    }
  }
  return 0.0;
}

IndexMotor::IndexMotor(control_loops::IndexLoop *my_index)
    : aos::control_loops::ControlLoop<control_loops::IndexLoop>(my_index),
      wrist_loop_(new IndexStateFeedbackLoop(MakeIndexLoop())),
      hopper_disc_count_(0),
      total_disc_count_(0),
      safe_goal_(Goal::HOLD),
      loader_goal_(LoaderGoal::READY),
      loader_state_(LoaderState::READY),
      loader_up_(false),
      disc_clamped_(false),
      disc_ejected_(false),
      last_bottom_disc_detect_(false),
      last_top_disc_detect_(false),
      no_prior_position_(true),
      missing_position_count_(0) {
}

/*static*/ const double IndexMotor::kTransferStartPosition = 0.0;
/*static*/ const double IndexMotor::kIndexStartPosition = 0.2159;
/*static*/ const double IndexMotor::kIndexFreeLength =
      IndexMotor::ConvertDiscAngleToDiscPosition((360 * 2 + 14) * M_PI / 180);
/*static*/ const double IndexMotor::kLoaderFreeStopPosition =
      kIndexStartPosition + kIndexFreeLength;
/*static*/ const double IndexMotor::kReadyToLiftPosition =
    kLoaderFreeStopPosition + 0.2921;
/*static*/ const double IndexMotor::kGrabberLength = 0.03175;
/*static*/ const double IndexMotor::kGrabberStartPosition =
    kReadyToLiftPosition - kGrabberLength;
/*static*/ const double IndexMotor::kGrabberMovementVelocity = 0.7;
/*static*/ const double IndexMotor::kLifterStopPosition =
    kReadyToLiftPosition + 0.161925;
/*static*/ const double IndexMotor::kLifterMovementVelocity = 1.0;
/*static*/ const double IndexMotor::kEjectorStopPosition =
    kLifterStopPosition + 0.01;
/*static*/ const double IndexMotor::kEjectorMovementVelocity = 1.0;
/*static*/ const double IndexMotor::kBottomDiscDetectStart = -0.08;
/*static*/ const double IndexMotor::kBottomDiscDetectStop = 0.200025;
/*static*/ const double IndexMotor::kBottomDiscIndexDelay = 0.01;

// TODO(aschuh): Figure these out.
/*static*/ const double IndexMotor::kTopDiscDetectStart =
    (IndexMotor::kLoaderFreeStopPosition -
     IndexMotor::ConvertDiscAngleToDiscPosition(60 * M_PI / 180));
// This is a guess for the width of the disc radially.  It should be close to 11
// inches but a bit below.
/*static*/ const double IndexMotor::kTopDiscDetectStop =
    IndexMotor::kTopDiscDetectStart + 10 * 0.0254;

const /*static*/ double IndexMotor::kDiscRadius = 10.875 * 0.0254 / 2;
const /*static*/ double IndexMotor::kRollerRadius = 2.0 * 0.0254 / 2;
const /*static*/ double IndexMotor::kTransferRollerRadius = 1.25 * 0.0254 / 2;

/*static*/ const int IndexMotor::kGrabbingDelay = 5;
/*static*/ const int IndexMotor::kLiftingDelay = 20;
/*static*/ const int IndexMotor::kShootingDelay = 5;
/*static*/ const int IndexMotor::kLoweringDelay = 20;

// TODO(aschuh): Tune these.
/*static*/ const double
    IndexMotor::IndexStateFeedbackLoop::kMinMotionVoltage = 5.0;
/*static*/ const double
    IndexMotor::IndexStateFeedbackLoop::kNoMotionCuttoffCount = 30;

// Distance to move the indexer when grabbing a disc.
const double kNextPosition = 10.0;

/*static*/ double IndexMotor::ConvertDiscAngleToIndex(const double angle) {
  return (angle * (1 + (kDiscRadius * 2 + kRollerRadius) / kRollerRadius));
}

/*static*/ double IndexMotor::ConvertDiscAngleToDiscPosition(
    const double angle) {
  return angle * (kDiscRadius + kRollerRadius);
}

/*static*/ double IndexMotor::ConvertDiscPositionToDiscAngle(
    const double position) {
  return position / (kDiscRadius + kRollerRadius);
}

/*static*/ double IndexMotor::ConvertIndexToDiscAngle(const double angle) {
  return (angle / (1 + (kDiscRadius * 2 + kRollerRadius) / kRollerRadius));
}

/*static*/ double IndexMotor::ConvertIndexToDiscPosition(const double angle) {
  return IndexMotor::ConvertDiscAngleToDiscPosition(
      ConvertIndexToDiscAngle(angle));
}

/*static*/ double IndexMotor::ConvertTransferToDiscPosition(
    const double angle) {
  const double gear_ratio =  (1 + (kDiscRadius * 2 + kTransferRollerRadius) /
                              kTransferRollerRadius);
  return angle / gear_ratio * (kDiscRadius + kTransferRollerRadius);
}

/*static*/ double IndexMotor::ConvertDiscPositionToIndex(
    const double position) {
  return IndexMotor::ConvertDiscAngleToIndex(
      ConvertDiscPositionToDiscAngle(position));
}

bool IndexMotor::MinDiscPosition(double *disc_position) {
  bool found_start = false;
  for (unsigned int i = 0; i < frisbees_.size(); ++i) {
    const Frisbee &frisbee = frisbees_[i];
    if (!found_start) {
      if (frisbee.has_position()) {
        *disc_position = frisbee.position();
        found_start = true;
      }
    } else {
      *disc_position = ::std::min(frisbee.position(),
                                  *disc_position);
    }
  }
  return found_start;
}

bool IndexMotor::MaxDiscPosition(double *disc_position) {
  bool found_start = false;
  for (unsigned int i = 0; i < frisbees_.size(); ++i) {
    const Frisbee &frisbee = frisbees_[i];
    if (!found_start) {
      if (frisbee.has_position()) {
        *disc_position = frisbee.position();
        found_start = true;
      }
    } else {
      *disc_position = ::std::max(frisbee.position(),
                                  *disc_position);
    }
  }
  return found_start;
}

void IndexMotor::IndexStateFeedbackLoop::CapU() {
  // If the voltage has been low for a large number of cycles, cut the motor
  // power.  This is generally very bad controls practice since this isn't LTI,
  // but we don't really care about tracking anything other than large step
  // inputs, and the loader doesn't need to be that accurate.
  if (::std::abs(U(0, 0)) < kMinMotionVoltage) {
    ++low_voltage_count_;
    if (low_voltage_count_ > kNoMotionCuttoffCount) {
      printf("Limiting power from %f to 0\n", U(0, 0));
      U(0, 0) = 0.0;
    }
  } else {
    low_voltage_count_ = 0;
  }

  for (int i = 0; i < kNumOutputs; ++i) {
    if (U[i] > plant.U_max[i]) {
      U[i] = plant.U_max[i];
    } else if (U[i] < plant.U_min[i]) {
      U[i] = plant.U_min[i];
    }
  }
}


// Positive angle is towards the shooter, and positive power is towards the
// shooter.
void IndexMotor::RunIteration(
    const control_loops::IndexLoop::Goal *goal,
    const control_loops::IndexLoop::Position *position,
    control_loops::IndexLoop::Output *output,
    control_loops::IndexLoop::Status *status) {
  // Make goal easy to work with and sanity check it.
  Goal goal_enum = static_cast<Goal>(goal->goal_state);
  if (goal->goal_state < 0 || goal->goal_state > 4) {
    LOG(ERROR, "Goal state is %d which is out of range.  Going to HOLD.\n",
        goal->goal_state);
    goal_enum = Goal::HOLD;
  }

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  double intake_voltage = 0.0;
  double transfer_voltage = 0.0;
  if (output) {
    output->intake_voltage = 0.0;
    output->transfer_voltage = 0.0;
    output->index_voltage = 0.0;
  }

  status->ready_to_intake = false;

  // Compute a safe index position that we can use.
  if (position) {
    wrist_loop_->Y << position->index_position;
    // Set the goal to be the current position if this is the first time through
    // so we don't always spin the indexer to the 0 position before starting.
    if (no_prior_position_) {
      wrist_loop_->R << wrist_loop_->Y(0, 0), 0.0;
      no_prior_position_ = false;
      last_bottom_disc_posedge_count_ = position->bottom_disc_posedge_count;
      last_bottom_disc_negedge_count_ = position->bottom_disc_negedge_count;
      last_bottom_disc_negedge_wait_count_ =
          position->bottom_disc_negedge_wait_count;
      last_top_disc_posedge_count_ = position->top_disc_posedge_count;
    }

    // If the cRIO is gone for 1/2 of a second, assume that it rebooted.
    if (missing_position_count_ > 50) {
      last_bottom_disc_posedge_count_ = position->bottom_disc_posedge_count;
      last_bottom_disc_negedge_count_ = position->bottom_disc_negedge_count;
      last_bottom_disc_negedge_wait_count_ =
          position->bottom_disc_negedge_wait_count;
      last_top_disc_posedge_count_ = position->top_disc_posedge_count;
      // Adjust the disc positions so that they don't have to move.
      const double disc_offset =
          position->index_position - wrist_loop_->X_hat(0, 0);
      for (auto frisbee = frisbees_.begin();
           frisbee != frisbees_.end(); ++frisbee) {
        frisbee->OffsetDisc(disc_offset);
      }
    }
    missing_position_count_ = 0;
  } else {
    ++missing_position_count_;
  }
  const double index_position = wrist_loop_->X_hat(0, 0);

  if (position) {
    if (!position->top_disc_detect) {
      // We don't see a disc.  Verify that there are no discs that we should be
      // seeing.
      // Assume that discs will move slow enough that we won't one as it goes
      // by.  They will either pile up above or below the sensor.

      double cumulative_offset = 0.0;
      for (auto frisbee = frisbees_.rbegin(), rend = frisbees_.rend();
           frisbee != rend; ++frisbee) {
        frisbee->OffsetDisc(cumulative_offset);
        double amount_moved = frisbee->ObserveNoTopDiscSensor(
            wrist_loop_->X_hat(0, 0), wrist_loop_->X_hat(1, 0));
        cumulative_offset += amount_moved;
      }
    }
    if (position->top_disc_posedge_count != last_top_disc_posedge_count_) {
      // TODO(aschuh): Sanity check this number...
      // Requires storing when the disc was last seen with the sensor off, and
      // figuring out what to do if things go south.

      // Find a disc that we should be seeing.  There are 3 cases...
      // 1) The top most disc is going up by the sensor.
      // 2) There is 1 disc almost in the loader, and past the sensor.
      //    This is the next disc.
      // 3) The top most disc is coming back down and we are seeing it.
      if (wrist_loop_->X_hat(1, 0) > 50.0) {
        // Moving up at a reasonable clip.
        // TODO(aschuh): Do something!
      } else if (wrist_loop_->X_hat(1, 0) < -50.0) {
        // Moving down at a reasonable clip.
        // Find the top disc and use that.
        // TODO(aschuh): Do something!
      } else {
        // TODO(aschuh): Do something!
      }
    }
  }

  // Bool to track if it is safe for the goal to change yet.
  bool safe_to_change_state_ = true;
  switch (safe_goal_) {
    case Goal::HOLD:
      // The goal should already be good, so sit tight with everything the same
      // as it was.
      break;
    case Goal::READY_LOWER:
    case Goal::INTAKE:
      {
        Time now = Time::Now();
        if (position) {
          // Posedge of the disc entering the beam break.
          if (position->bottom_disc_posedge_count !=
              last_bottom_disc_posedge_count_) {
            transfer_frisbee_.Reset();
            transfer_frisbee_.bottom_posedge_time_ = now;
            printf("Posedge of bottom disc %f\n",
                   transfer_frisbee_.bottom_posedge_time_.ToSeconds());
            ++hopper_disc_count_;
            ++total_disc_count_;
          }

          // Disc exited the beam break now.
          if (position->bottom_disc_negedge_count !=
              last_bottom_disc_negedge_count_) {
            transfer_frisbee_.bottom_negedge_time_ = now;
            printf("Negedge of bottom disc %f\n",
                   transfer_frisbee_.bottom_negedge_time_.ToSeconds());
            frisbees_.push_front(transfer_frisbee_);
          }

          if (position->bottom_disc_detect) {
            intake_voltage = transfer_voltage = 12.0;
            // Must wait until the disc gets out before we can change state.
            safe_to_change_state_ = false;

            // TODO(aschuh): A disc on the way through needs to start moving
            // the indexer if it isn't already moving.  Maybe?

            Time elapsed_posedge_time = now -
                transfer_frisbee_.bottom_posedge_time_;
            if (elapsed_posedge_time >= Time::InSeconds(0.3)) {
              // It has been too long.  The disc must be jammed.
              LOG(ERROR, "Been way too long.  Jammed disc?\n");
              printf("Been way too long.  Jammed disc?\n");
            }
          }

          // Check all non-indexed discs and see if they should be indexed.
          for (auto frisbee = frisbees_.begin();
               frisbee != frisbees_.end(); ++frisbee) {
            if (!frisbee->has_been_indexed_) {
              intake_voltage = transfer_voltage = 12.0;

              if (last_bottom_disc_negedge_wait_count_ !=
                  position->bottom_disc_negedge_wait_count) {
                // We have an index difference.
                // Save the indexer position, and the time.
                if (last_bottom_disc_negedge_wait_count_ + 1 !=
                  position->bottom_disc_negedge_wait_count) {
                  LOG(ERROR, "Funny, we got 2 edges since we last checked.\n");
                }

                // Save the captured position as the position at which the disc
                // touched the indexer.
                LOG(INFO, "Grabbed on the index now at %f\n", index_position);
                printf("Grabbed on the index now at %f\n", index_position);
                frisbee->has_been_indexed_ = true;
                frisbee->index_start_position_ =
                    position->bottom_disc_negedge_wait_position;
              }
            }
            if (!frisbee->has_been_indexed_) {
              // All discs must be indexed before it is safe to stop indexing.
              safe_to_change_state_ = false;
            }
          }

          // Figure out where the indexer should be to move the discs down to
          // the right position.
          double max_disc_position;
          if (MaxDiscPosition(&max_disc_position)) {
            printf("There is a disc down here!\n");
            // TODO(aschuh): Figure out what to do if grabbing the next one
            // would cause things to jam into the loader.
            // Say we aren't ready any more.  Undefined behavior will result if
            // that isn't observed.
            double bottom_disc_position =
                max_disc_position + ConvertDiscAngleToIndex(M_PI);
            wrist_loop_->R << bottom_disc_position, 0.0;

            // Verify that we are close enough to the goal so that we should be
            // fine accepting the next disc.
            double disc_error_meters = ConvertIndexToDiscPosition(
                wrist_loop_->X_hat(0, 0) - bottom_disc_position);
            // We are ready for the next disc if the first one is in the first
            // half circle of the indexer.  It will take time for the disc to
            // come into the indexer, so we will be able to move it out of the
            // way in time.
            // This choice also makes sure that we don't claim that we aren't
            // ready between full speed intaking.
            if (-ConvertDiscAngleToIndex(M_PI) < disc_error_meters &&
                disc_error_meters < 0.04) {
              // We are only ready if we aren't being asked to change state or
              // are full.
              status->ready_to_intake =
                  (safe_goal_ == goal_enum) && hopper_disc_count_ < 4;
            } else {
              status->ready_to_intake = false;
            }
          } else {
            // No discs!  We are always ready for more if we aren't being
            // asked to change state.
            status->ready_to_intake = (safe_goal_ == goal_enum);
          }

          // Turn on the transfer roller if we are ready.
          if (status->ready_to_intake && hopper_disc_count_ < 4 &&
              safe_goal_ == Goal::INTAKE) {
            intake_voltage = transfer_voltage = 12.0;
          }
        }
        printf("INTAKE\n");
      }
      break;
    case Goal::READY_SHOOTER:
    case Goal::SHOOT:
      // Check if we have any discs to shoot or load and handle them.
      double min_disc_position;
      if (MinDiscPosition(&min_disc_position)) {
        const double ready_disc_position =
            min_disc_position + ConvertDiscPositionToIndex(kIndexFreeLength) -
            ConvertDiscAngleToIndex(M_PI / 6.0);

        const double grabbed_disc_position =
            min_disc_position +
            ConvertDiscPositionToIndex(kReadyToLiftPosition -
                                       kIndexStartPosition + 0.03);

        // Check the state of the loader FSM.
        // If it is ready to load discs, position the disc so that it is ready
        // to be grabbed.
        // If it isn't ready, there is a disc in there.  It needs to finish it's
        // cycle first.
        if (loader_state_ != LoaderState::READY) {
          // We already have a disc in the loader.
          // Stage the discs back a bit.
          wrist_loop_->R << ready_disc_position, 0.0;

          // Shoot if we are grabbed and being asked to shoot.
          if (loader_state_ == LoaderState::GRABBED &&
              safe_goal_ == Goal::SHOOT) {
            loader_goal_ = LoaderGoal::SHOOT_AND_RESET;
          }

          // Must wait until it has been grabbed to continue.
          if (loader_state_ == LoaderState::GRABBING) {
            safe_to_change_state_ = false;
          }
        } else {
          // No disc up top right now.
          wrist_loop_->R << grabbed_disc_position, 0.0;

          // See if the disc has gotten pretty far up yet.
          if (wrist_loop_->X_hat(0, 0) > ready_disc_position) {
            // Point of no return.  We are committing to grabbing it now.
            safe_to_change_state_ = false;
            const double robust_grabbed_disc_position =
                (grabbed_disc_position -
                 ConvertDiscPositionToIndex(kGrabberLength));

            // If close, start grabbing and/or shooting.
            if (wrist_loop_->X_hat(0, 0) > robust_grabbed_disc_position) {
              // Start the state machine.
              if (safe_goal_ == Goal::SHOOT) {
                loader_goal_ = LoaderGoal::SHOOT_AND_RESET;
              } else {
                loader_goal_ = LoaderGoal::GRAB;
              }
              // This frisbee is now gone.  Take it out of the queue.
              frisbees_.pop_back();
              --hopper_disc_count_;
            }
          }
        }
      }

      printf("READY_SHOOTER or SHOOT\n");
      break;
  }

  // The only way out of the loader is to shoot the disc.  The FSM can only go
  // forwards.
  switch (loader_state_) {
    case LoaderState::READY:
      printf("Loader READY\n");
      // Open and down, ready to accept a disc.
      loader_up_ = false;
      disc_clamped_ = false;
      disc_ejected_ = false;
      if (loader_goal_ == LoaderGoal::GRAB ||
          loader_goal_ == LoaderGoal::SHOOT_AND_RESET) {
        if (loader_goal_ == LoaderGoal::GRAB) {
          printf("Told to GRAB, moving on\n");
        } else {
          printf("Told to SHOOT_AND_RESET, moving on\n");
        }
        loader_state_ = LoaderState::GRABBING;
        loader_countdown_ = kGrabbingDelay;
      } else {
        break;
      }
    case LoaderState::GRABBING:
      printf("Loader GRABBING %d\n", loader_countdown_);
      // Closing the grabber.
      loader_up_ = false;
      disc_clamped_ = true;
      disc_ejected_ = false;
      if (loader_countdown_ > 0) {
        --loader_countdown_;
        break;
      } else {
        loader_state_ = LoaderState::GRABBED;
      }
    case LoaderState::GRABBED:
      printf("Loader GRABBED\n");
      // Grabber closed.
      loader_up_ = false;
      disc_clamped_ = true;
      disc_ejected_ = false;
      if (loader_goal_ == LoaderGoal::SHOOT_AND_RESET) {
        // TODO(aschuh): Only shoot if the shooter is up to speed.
        // Seems like that would have us shooting a bit later than we could be,
        // but it also probably spins back up real fast.
        loader_state_ = LoaderState::LIFTING;
        loader_countdown_ = kLiftingDelay;
        printf("Told to SHOOT_AND_RESET, moving on\n");
      } else if (loader_goal_ == LoaderGoal::READY) {
        LOG(ERROR, "Can't go to ready when we have something grabbed.\n");
        printf("Can't go to ready when we have something grabbed.\n");
        break;
      } else {
        break;
      }
    case LoaderState::LIFTING:
      printf("Loader LIFTING %d\n", loader_countdown_);
      // Lifting the disc.
      loader_up_ = true;
      disc_clamped_ = true;
      disc_ejected_ = false;
      if (loader_countdown_ > 0) {
        --loader_countdown_;
        break;
      } else {
        loader_state_ = LoaderState::LIFTED;
      }
    case LoaderState::LIFTED:
      printf("Loader LIFTED\n");
      // Disc lifted.  Time to eject it out.
      loader_up_ = true;
      disc_clamped_ = true;
      disc_ejected_ = false;
      loader_state_ = LoaderState::SHOOTING;
      loader_countdown_ = kShootingDelay;
    case LoaderState::SHOOTING:
      printf("Loader SHOOTING %d\n", loader_countdown_);
      // Ejecting the disc into the shooter.
      loader_up_ = true;
      disc_clamped_ = false;
      disc_ejected_ = true;
      if (loader_countdown_ > 0) {
        --loader_countdown_;
        break;
      } else {
        loader_state_ = LoaderState::SHOOT;
      }
    case LoaderState::SHOOT:
      printf("Loader SHOOT\n");
      // The disc has been shot.
      loader_up_ = true;
      disc_clamped_ = false;
      disc_ejected_ = true;
      loader_state_ = LoaderState::LOWERING;
      loader_countdown_ = kLoweringDelay;
    case LoaderState::LOWERING:
      printf("Loader LOWERING %d\n", loader_countdown_);
      // Lowering the loader back down.
      loader_up_ = false;
      disc_clamped_ = false;
      disc_ejected_ = true;
      if (loader_countdown_ > 0) {
        --loader_countdown_;
        break;
      } else {
        loader_state_ = LoaderState::LOWERED;
      }
    case LoaderState::LOWERED:
      printf("Loader LOWERED\n");
      // The indexer is lowered.
      loader_up_ = false;
      disc_clamped_ = false;
      disc_ejected_ = false;
      loader_state_ = LoaderState::READY;
      // Once we have shot, we need to hang out in READY until otherwise
      // notified.
      loader_goal_ = LoaderGoal::READY;
      break;
  }

  // Update the observer.
  wrist_loop_->Update(position != NULL, output == NULL);

  if (position) {
    LOG(DEBUG, "pos=%f\n", position->index_position);
    last_bottom_disc_detect_ = position->bottom_disc_detect;
    last_top_disc_detect_ = position->top_disc_detect;
    last_bottom_disc_posedge_count_ = position->bottom_disc_posedge_count;
    last_bottom_disc_negedge_count_ = position->bottom_disc_negedge_count;
    last_bottom_disc_negedge_wait_count_ =
        position->bottom_disc_negedge_wait_count;
    last_top_disc_posedge_count_ = position->top_disc_posedge_count;
  }

  status->hopper_disc_count = hopper_disc_count_;
  status->total_disc_count = total_disc_count_;
  status->preloaded = (loader_state_ != LoaderState::READY);

  if (output) {
    output->intake_voltage = intake_voltage;
    output->transfer_voltage = transfer_voltage;
    output->index_voltage = wrist_loop_->U(0, 0);
    output->loader_up = loader_up_;
    output->disc_clamped = disc_clamped_;
    output->disc_ejected = disc_ejected_;
  }

  if (safe_to_change_state_) {
    safe_goal_ = goal_enum;
  }
}

}  // namespace control_loops
}  // namespace frc971
