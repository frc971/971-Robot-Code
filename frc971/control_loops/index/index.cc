#include "frc971/control_loops/index/index.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/inttypes.h"

#include "frc971/constants.h"
#include "frc971/control_loops/index/index_motor_plant.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"

using ::aos::time::Time;

namespace frc971 {
namespace control_loops {

double IndexMotor::Frisbee::ObserveNoTopDiscSensor(double index_position) {
  // The absolute disc position in meters.
  double disc_position = absolute_position(index_position);
  if (IndexMotor::kTopDiscDetectStart <= disc_position &&
      disc_position <= IndexMotor::kTopDiscDetectStop) {
    // Whoops, this shouldn't be happening.
    // Move the disc off the way that makes most sense.
    double distance_to_above = IndexMotor::ConvertDiscPositionToIndex(
        ::std::abs(disc_position - IndexMotor::kTopDiscDetectStop));
    double distance_to_below = IndexMotor::ConvertDiscPositionToIndex(
        ::std::abs(disc_position - IndexMotor::kTopDiscDetectStart));
    if (distance_to_above < distance_to_below) {
      LOG(INFO, "Moving disc to top slow.\n");
      // Move it up.
      index_start_position_ -= distance_to_above;
      return -distance_to_above;
    } else {
      LOG(INFO, "Moving disc to bottom slow.\n");
      index_start_position_ += distance_to_below;
      return distance_to_below;
    }
  }
  return 0.0;
}

IndexMotor::IndexMotor(control_loops::IndexLoop *my_index)
    : aos::control_loops::ControlLoop<control_loops::IndexLoop>(my_index),
      wrist_loop_(new IndexStateFeedbackLoop(MakeIndexLoop())),
      hopper_disc_count_(0),
      total_disc_count_(0),
      shot_disc_count_(0),
      safe_goal_(Goal::HOLD),
      loader_goal_(LoaderGoal::READY),
      loader_state_(LoaderState::READY),
      loader_up_(false),
      disc_clamped_(false),
      disc_ejected_(false),
      is_shooting_(false),
      last_bottom_disc_detect_(false),
      last_top_disc_detect_(false),
      hopper_clear_(true),
      no_prior_position_(true),
      missing_position_count_(0) {
}

/*static*/ const double IndexMotor::kTransferStartPosition = 0.0;
/*static*/ const double IndexMotor::kIndexStartPosition = 0.2159;
/*static*/ const double IndexMotor::kIndexFreeLength =
      IndexMotor::ConvertDiscAngleToDiscPosition((360 * 2 + 14) * M_PI / 180);
/*static*/ const double IndexMotor::kLoaderFreeStopPosition =
      kIndexStartPosition + kIndexFreeLength;
/*static*/ const double IndexMotor::kReadyToPreload =
      kLoaderFreeStopPosition - ConvertDiscAngleToDiscPosition(M_PI / 6.0);
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
/*static*/ const double IndexMotor::kBottomDiscDetectStart = 0.00;
/*static*/ const double IndexMotor::kBottomDiscDetectStop = 0.13;
/*static*/ const double IndexMotor::kBottomDiscIndexDelay = 0.032;
/*static*/ const ::aos::time::Time IndexMotor::kTransferOffDelay =
    ::aos::time::Time::InSeconds(0.3);

// TODO(aschuh): Verify these with the sensor actually on.
/*static*/ const double IndexMotor::kTopDiscDetectStart =
    (IndexMotor::kLoaderFreeStopPosition -
     IndexMotor::ConvertDiscAngleToDiscPosition(49 * M_PI / 180));
/*static*/ const double IndexMotor::kTopDiscDetectStop =
    (IndexMotor::kLoaderFreeStopPosition +
     IndexMotor::ConvertDiscAngleToDiscPosition(19 * M_PI / 180));

// I measured the angle between 2 discs.  That then gives me the distance
// between 2 posedges (or negedges).  Then subtract off the width of the
// positive pulse, and that gives the width of the negative pulse.
/*static*/ const double IndexMotor::kTopDiscDetectMinSeperation =
    (IndexMotor::ConvertDiscAngleToDiscPosition(120 * M_PI / 180) -
     (IndexMotor::kTopDiscDetectStop - IndexMotor::kTopDiscDetectStart));

const /*static*/ double IndexMotor::kDiscRadius = 10.875 * 0.0254 / 2;
const /*static*/ double IndexMotor::kRollerRadius = 2.0 * 0.0254 / 2;
const /*static*/ double IndexMotor::kTransferRollerRadius = 1.25 * 0.0254 / 2;

/*static*/ const int IndexMotor::kGrabbingDelay = 5;
/*static*/ const int IndexMotor::kLiftingDelay = 2;
/*static*/ const int IndexMotor::kLiftingTimeout = 100;
/*static*/ const int IndexMotor::kShootingDelay = 10;
/*static*/ const int IndexMotor::kLoweringDelay = 4;
/*static*/ const int IndexMotor::kLoweringTimeout = 120;

// TODO(aschuh): Tune these.
/*static*/ const double
    IndexMotor::IndexStateFeedbackLoop::kMinMotionVoltage = 11.0;
/*static*/ const double
    IndexMotor::IndexStateFeedbackLoop::kNoMotionCuttoffCount = 20;

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

bool IndexMotor::MinDiscPosition(double *disc_position, Frisbee **found_disc) {
  bool found_start = false;
  for (unsigned int i = 0; i < frisbees_.size(); ++i) {
    Frisbee &frisbee = frisbees_[i];
    if (!found_start) {
      if (frisbee.has_position()) {
        *disc_position = frisbee.position();
        if (found_disc) {
          *found_disc = &frisbee;
        }
        found_start = true;
      }
    } else {
      if (frisbee.position() <= *disc_position) {
        *disc_position = frisbee.position();
        if (found_disc) {
          *found_disc = &frisbee;
        }
      }
    }
  }
  return found_start;
}

bool IndexMotor::MaxDiscPosition(double *disc_position, Frisbee **found_disc) {
  bool found_start = false;
  for (unsigned int i = 0; i < frisbees_.size(); ++i) {
    Frisbee &frisbee = frisbees_[i];
    if (!found_start) {
      if (frisbee.has_position()) {
        *disc_position = frisbee.position();
        if (found_disc) {
          *found_disc = &frisbee;
        }
        found_start = true;
      }
    } else {
      if (frisbee.position() > *disc_position) {
        *disc_position = frisbee.position();
        if (found_disc) {
          *found_disc = &frisbee;
        }
      }
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
      U(0, 0) = 0.0;
    }
  } else {
    low_voltage_count_ = 0;
  }

  for (int i = 0; i < kNumOutputs; ++i) {
    if (U(i, 0) > U_max(i, 0)) {
      U(i, 0) = U_max(i, 0);
    } else if (U(i, 0) < U_min(i, 0)) {
      U(i, 0) = U_min(i, 0);
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
  Time now = Time::Now();
  // Make goal easy to work with and sanity check it.
  Goal goal_enum = static_cast<Goal>(goal->goal_state);
  if (goal->goal_state < 0 || goal->goal_state > 5) {
    LOG(ERROR,
        "Goal state is %" PRId32 " which is out of range.  Going to HOLD.\n",
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

  // Set the controller to use to be the one designed for the current number of
  // discs in the hopper.  This is safe since the controller prevents the index
  // from being set out of bounds and picks the closest controller.
  wrist_loop_->set_controller_index(frisbees_.size());

  // Compute a safe index position that we can use.
  if (position) {
    wrist_loop_->Y << position->index_position;
    // Set the goal to be the current position if this is the first time through
    // so we don't always spin the indexer to the 0 position before starting.
    if (no_prior_position_) {
      LOG(INFO, "no prior position; resetting\n");
      wrist_loop_->R << wrist_loop_->Y(0, 0), 0.0;
      wrist_loop_->X_hat(0, 0) = wrist_loop_->Y(0, 0);
      no_prior_position_ = false;
      last_bottom_disc_posedge_count_ = position->bottom_disc_posedge_count;
      last_bottom_disc_negedge_count_ = position->bottom_disc_negedge_count;
      last_bottom_disc_negedge_wait_count_ =
          position->bottom_disc_negedge_wait_count;
      last_top_disc_posedge_count_ = position->top_disc_posedge_count;
      last_top_disc_negedge_count_ = position->top_disc_negedge_count;
      last_top_disc_detect_ = position->top_disc_detect;
      // The open positions for the upper is right here and isn't a hard edge.
      upper_open_region_.Restart(wrist_loop_->Y(0, 0));
      lower_open_region_.Restart(wrist_loop_->Y(0, 0));
    }

    // If the cRIO is gone for over 1/2 of a second, assume that it rebooted.
    if (missing_position_count_ > 50) {
      LOG(INFO, "assuming cRIO rebooted\n");
      last_bottom_disc_posedge_count_ = position->bottom_disc_posedge_count;
      last_bottom_disc_negedge_count_ = position->bottom_disc_negedge_count;
      last_bottom_disc_negedge_wait_count_ =
          position->bottom_disc_negedge_wait_count;
      last_top_disc_posedge_count_ = position->top_disc_posedge_count;
      last_top_disc_negedge_count_ = position->top_disc_negedge_count;
      last_top_disc_detect_ = position->top_disc_detect;
      // We can't really trust the open range any more if the crio rebooted.
      upper_open_region_.Restart(wrist_loop_->Y(0, 0));
      lower_open_region_.Restart(wrist_loop_->Y(0, 0));
      // Adjust the disc positions so that they don't have to move.
      const double disc_offset =
          position->index_position - wrist_loop_->X_hat(0, 0);
      for (auto frisbee = frisbees_.begin();
           frisbee != frisbees_.end(); ++frisbee) {
        frisbee->OffsetDisc(disc_offset);
      }
      wrist_loop_->X_hat(0, 0) = wrist_loop_->Y(0, 0);
    }
    missing_position_count_ = 0;
    if (last_top_disc_detect_) {
      if (last_top_disc_posedge_count_ != position->top_disc_posedge_count) {
        LOG(INFO, "Ignoring a top disc posedge\n");
      }
      last_top_disc_posedge_count_ = position->top_disc_posedge_count;
    }
    if (!last_top_disc_detect_) {
      if (last_top_disc_negedge_count_ != position->top_disc_negedge_count) {
        LOG(INFO, "Ignoring a top disc negedge\n");
      }
      last_top_disc_negedge_count_ = position->top_disc_negedge_count;
    }
  } else {
    ++missing_position_count_;
  }
  const double index_position = wrist_loop_->X_hat(0, 0);

  if (position) {
    // Reset the open region if we saw a negedge.
    if (position->bottom_disc_negedge_wait_count !=
        last_bottom_disc_negedge_wait_count_) {
      // Saw a negedge, must be a new region.
      lower_open_region_.Restart(position->bottom_disc_negedge_wait_position);
    }
    // Reset the open region if we saw a negedge.
    if (position->top_disc_negedge_count != last_top_disc_negedge_count_) {
      // Saw a negedge, must be a new region.
      upper_open_region_.Restart(position->top_disc_negedge_position);
    }

    // No disc.  Expand the open region.
    if (!position->bottom_disc_detect) {
      lower_open_region_.Expand(index_position);
    }

    // No disc.  Expand the open region.
    if (!position->top_disc_detect) {
      upper_open_region_.Expand(index_position);
    }

    if (!position->top_disc_detect) {
      // We don't see a disc.  Verify that there are no discs that we should be
      // seeing.
      // Assume that discs will move slow enough that we won't miss one as it
      // goes by.  They will either pile up above or below the sensor.

      double cumulative_offset = 0.0;
      for (auto frisbee = frisbees_.rbegin(), rend = frisbees_.rend();
           frisbee != rend; ++frisbee) {
        frisbee->OffsetDisc(cumulative_offset);
        double amount_moved = frisbee->ObserveNoTopDiscSensor(
            wrist_loop_->X_hat(0, 0));
        cumulative_offset += amount_moved;
      }
    }

    if (position->top_disc_posedge_count != last_top_disc_posedge_count_) {
      LOG(INFO, "Saw a top posedge\n");
      const double index_position = wrist_loop_->X_hat(0, 0) -
          position->index_position + position->top_disc_posedge_position;
      // TODO(aschuh): Sanity check this number...
      // Requires storing when the disc was last seen with the sensor off, and
      // figuring out what to do if things go south.

      // 1 if discs are going up, 0 if we have no clue, and -1 if they are going
      // down.
      int disc_direction = 0;
      if (wrist_loop_->X_hat(1, 0) > 100.0) {
        disc_direction = 1;
      } else if (wrist_loop_->X_hat(1, 0) < -100.0) {
        disc_direction = -1;
      } else {
        // Save the upper and lower positions that we last saw a disc at.
        // If there is a big buffer above, must be a disc from below.
        // If there is a big buffer below, must be a disc from above.
        // This should work to replace the velocity threshold above.

        const double open_width = upper_open_region_.width();
        const double relative_upper_open_precentage =
            (upper_open_region_.upper_bound() - index_position) / open_width;
        const double relative_lower_open_precentage =
            (index_position - upper_open_region_.lower_bound()) / open_width;

        if (ConvertIndexToDiscPosition(open_width) <
            kTopDiscDetectMinSeperation * 0.9) {
          LOG(ERROR, "Discs are way too close to each other.  Doing nothing\n");
        } else if (relative_upper_open_precentage > 0.75) {
          // Looks like it is a disc going down from above since we are near
          // the upper edge.
          disc_direction = -1;
          LOG(INFO, "Disc edge going down\n");
        } else if (relative_lower_open_precentage > 0.75) {
          // Looks like it is a disc going up from below since we are near
          // the lower edge.
          disc_direction = 1;
          LOG(INFO, "Disc edge going up\n");
        } else {
          LOG(ERROR,
              "Got an edge in the middle of what should be an open region.\n");
          LOG(ERROR, "Open width: %f upper precentage %f %%\n",
              open_width, relative_upper_open_precentage);
        }
      }

      if (disc_direction > 0) {
        // Moving up at a reasonable clip.
        // Find the highest disc that is below the top disc sensor.
        // While we are at it, count the number above and log an error if there
        // are too many.
        if (frisbees_.size() == 0) {
          Frisbee new_frisbee;
          new_frisbee.has_been_indexed_ = true;
          new_frisbee.index_start_position_ = index_position -
              ConvertDiscPositionToIndex(kTopDiscDetectStart -
                                         kIndexStartPosition);
          ++hopper_disc_count_;
          ++total_disc_count_;
          frisbees_.push_front(new_frisbee);
          LOG(WARNING, "Added a disc to the hopper at the top sensor\n");
        }

        int above_disc_count = 0;
        double highest_position = 0;
        Frisbee *highest_frisbee_below_sensor = NULL;
        for (auto frisbee = frisbees_.rbegin(), rend = frisbees_.rend();
             frisbee != rend; ++frisbee) {
          const double disc_position = frisbee->absolute_position(
              index_position);
          // It is save to use the top position for the cuttoff, since the
          // sensor being low will result in discs being pushed off of it.
          if (disc_position >= kTopDiscDetectStop) {
            ++above_disc_count;
          } else if (!highest_frisbee_below_sensor ||
                     disc_position > highest_position) {
            highest_frisbee_below_sensor = &*frisbee;
            highest_position = disc_position;
          }
        }

        if (!highest_frisbee_below_sensor) {
          Frisbee new_frisbee;
          new_frisbee.has_been_indexed_ = true;
          new_frisbee.index_start_position_ = index_position -
              ConvertDiscPositionToIndex(kTopDiscDetectStart -
                                         kIndexStartPosition);
          highest_position = kTopDiscDetectStart;
          ++hopper_disc_count_;
          ++total_disc_count_;
          frisbees_.push_front(new_frisbee);
          LOG(WARNING, "Added a disc to the hopper at the top sensor because the one we know about is up top\n");
        }

        if (above_disc_count > 1) {
          LOG(ERROR, "We have 2 discs above the top sensor.\n");
        }
        // We now have the disc.  Shift all the ones below the sensor up by the
        // computed delta.
        const double disc_delta = IndexMotor::ConvertDiscPositionToIndex(
            highest_position - kTopDiscDetectStart);
        for (auto frisbee = frisbees_.rbegin(), rend = frisbees_.rend();
             frisbee != rend; ++frisbee) {
          const double disc_position = frisbee->absolute_position(
              index_position);
          if (disc_position < kTopDiscDetectStop) {
            LOG(INFO, "Moving disc down by %f meters, since it is at %f and top is [%f, %f]\n",
                ConvertIndexToDiscPosition(disc_delta),
                disc_position, kTopDiscDetectStart,
                kTopDiscDetectStop);
            frisbee->OffsetDisc(disc_delta);
          }
        }
        if (highest_frisbee_below_sensor) {
          LOG(INFO, "Currently have %d discs, saw posedge moving up.  "
              "Moving down by %f to %f\n", frisbees_.size(),
              ConvertIndexToDiscPosition(disc_delta),
              highest_frisbee_below_sensor->absolute_position(
                  wrist_loop_->X_hat(0, 0)));
        } else {
          LOG(INFO, "Currently have %d discs, saw posedge moving up.  "
              "Moving down by %f\n", frisbees_.size(),
              ConvertIndexToDiscPosition(disc_delta));
        }
      } else if (disc_direction < 0) {
        // Moving down at a reasonable clip.
        // There can only be 1 disc up top that would give us a posedge.
        // Find it and place it at the one spot that it can be.
        double min_disc_position = 0;
        Frisbee *min_frisbee = NULL;
        MinDiscPosition(&min_disc_position, &min_frisbee);
        if (!min_frisbee) {
          // Uh, oh, we see a disc but there isn't one...
          LOG(ERROR, "Saw a disc up top but there isn't one in the hopper\n");
        } else {
          const double disc_position = min_frisbee->absolute_position(
              index_position);

          const double disc_delta_meters = disc_position - kTopDiscDetectStop;
          const double disc_delta = IndexMotor::ConvertDiscPositionToIndex(
              disc_delta_meters);
          LOG(INFO, "Posedge going down.  Moving top disc down by %f\n",
              disc_delta_meters);
          for (auto frisbee = frisbees_.begin(), end = frisbees_.end();
               frisbee != end; ++frisbee) {
            frisbee->OffsetDisc(disc_delta);
          }
        }
      } else {
        LOG(ERROR, "Not sure how to handle the upper posedge, doing nothing\n");
      }
    }
  }

  // Bool to track if it is safe for the goal to change yet.
  bool safe_to_change_state = true;
  if (!position) {
    // This fixes a nasty indexer bug.
    // If we didn't get a position this cycle, we don't run the code below which
    // checks the state of the disc detect sensor and whether all the discs are
    // indexed.  It is therefore not safe to change state and loose track of
    // that disc.
    safe_to_change_state = false;
  }
  switch (safe_goal_) {
    case Goal::HOLD:
      // The goal should already be good, so sit tight with everything the same
      // as it was.
      break;
    case Goal::READY_LOWER:
    case Goal::INTAKE:
      hopper_clear_ = false;
      {
        if (position) {
          // Posedge of the disc entering the beam break.
          if (position->bottom_disc_posedge_count !=
              last_bottom_disc_posedge_count_) {
            transfer_frisbee_.Reset();
            transfer_frisbee_.bottom_posedge_time_ = now;
            LOG(INFO, "Posedge of bottom disc %f\n",
                transfer_frisbee_.bottom_posedge_time_.ToSeconds());
            ++hopper_disc_count_;
            ++total_disc_count_;
          }

          // Disc exited the beam break now.
          if (position->bottom_disc_negedge_count !=
              last_bottom_disc_negedge_count_) {
            transfer_frisbee_.bottom_negedge_time_ = now;
            LOG(INFO, "Negedge of bottom disc %f\n",
                transfer_frisbee_.bottom_negedge_time_.ToSeconds());
            frisbees_.push_front(transfer_frisbee_);
          }

          if (position->bottom_disc_detect) {
            intake_voltage = 0.0;
            transfer_voltage = 12.0;
            // Must wait until the disc gets out before we can change state.
            safe_to_change_state = false;

            // TODO(aschuh): A disc on the way through needs to start moving
            // the indexer if it isn't already moving.  Maybe?

            Time elapsed_posedge_time = now -
                transfer_frisbee_.bottom_posedge_time_;
            if (elapsed_posedge_time >= Time::InSeconds(0.3)) {
              // It has been too long.  The disc must be jammed.
              LOG(ERROR, "Been way too long.  Jammed disc?\n");
              intake_voltage = -12.0;
              transfer_voltage = -12.0;
            }
          }

          // Check all non-indexed discs and see if they should be indexed.
          for (auto frisbee = frisbees_.begin();
               frisbee != frisbees_.end(); ++frisbee) {
            if (!frisbee->has_been_indexed_) {
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
                frisbee->has_been_indexed_ = true;
                frisbee->index_start_position_ =
                    position->bottom_disc_negedge_wait_position;
              }
            }
          }
        }
        for (auto frisbee = frisbees_.begin();
             frisbee != frisbees_.end(); ++frisbee) {
          if (!frisbee->has_been_indexed_) {
            intake_voltage = 0.0;
            transfer_voltage = 12.0;

            // All discs must be indexed before it is safe to stop indexing.
            safe_to_change_state = false;
          }
        }

        // Figure out where the indexer should be to move the discs down to
        // the right position.
        double max_disc_position = 0;
        if (MaxDiscPosition(&max_disc_position, NULL)) {
          LOG(DEBUG, "There is a disc down here!\n");
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
      LOG(DEBUG, "INTAKE\n");
      break;
    case Goal::REINITIALIZE:
      LOG(WARNING, "Reinitializing the indexer\n");
      break;
    case Goal::READY_SHOOTER:
    case Goal::SHOOT:
      // Don't let us leave the shoot or preload state if there are 4 discs in
      // the hopper.
      if (hopper_disc_count_ >= 4 && goal_enum != Goal::SHOOT) {
        safe_to_change_state = false;
      }
      // Check if we have any discs to shoot or load and handle them.
      double min_disc_position = 0;
      if (MinDiscPosition(&min_disc_position, NULL)) {
        const double ready_disc_position = min_disc_position +
            ConvertDiscPositionToIndex(kReadyToPreload - kIndexStartPosition);

        const double grabbed_disc_position =
            min_disc_position +
            ConvertDiscPositionToIndex(kReadyToLiftPosition -
                                       kIndexStartPosition + 0.07);

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
            is_shooting_ = true;
          }

          // Must wait until it has been grabbed to continue.
          if (loader_state_ == LoaderState::GRABBING) {
            safe_to_change_state = false;
          }
        } else {
          // No disc up top right now.
          wrist_loop_->R << grabbed_disc_position, 0.0;

          // See if the disc has gotten pretty far up yet.
          if (wrist_loop_->X_hat(0, 0) > ready_disc_position) {
            // Point of no return.  We are committing to grabbing it now.
            safe_to_change_state = false;
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
            }
          }
        }
      } else {
        if (loader_state_ != LoaderState::READY) {
          // Shoot if we are grabbed and being asked to shoot.
          if (loader_state_ == LoaderState::GRABBED &&
              safe_goal_ == Goal::SHOOT) {
            loader_goal_ = LoaderGoal::SHOOT_AND_RESET;
          }
        } else {
          // Ok, no discs in sight.  Spin the hopper up by 150% of it's full
          // range and verify that we don't see anything.
          const double hopper_clear_verification_position =
              ::std::max(upper_open_region_.lower_bound(),
                         lower_open_region_.lower_bound()) +
              ConvertDiscPositionToIndex(kIndexFreeLength) * 1.5;

          wrist_loop_->R << hopper_clear_verification_position, 0.0;
          if (::std::abs(wrist_loop_->X_hat(0, 0) -
                         hopper_clear_verification_position) <
              ConvertDiscPositionToIndex(0.05)) {
            // We are at the end of the range.  There are no more discs here.
            while (frisbees_.size() > 0) {
              LOG(ERROR, "Dropping an extra disc since it can't exist\n");
              LOG(ERROR, "Upper is [%f %f]\n",
                  upper_open_region_.upper_bound(),
                  upper_open_region_.lower_bound());
              LOG(ERROR, "Lower is [%f %f]\n",
                  lower_open_region_.upper_bound(),
                  lower_open_region_.lower_bound());
              frisbees_.pop_back();
              --hopper_disc_count_;
              --total_disc_count_;
            }
            if (hopper_disc_count_ != 0) {
              LOG(ERROR,
                  "Emptied the hopper out but there are still discs there\n");
              hopper_disc_count_ = 0;
            }
            hopper_clear_ = true;
          }
        }
      }

      {
        const double hopper_clear_verification_position =
            ::std::max(upper_open_region_.lower_bound(),
                       lower_open_region_.lower_bound()) +
            ConvertDiscPositionToIndex(kIndexFreeLength) * 1.5;

        if (wrist_loop_->X_hat(0, 0) >
            hopper_clear_verification_position +
            ConvertDiscPositionToIndex(0.05)) {
          // We are at the end of the range.  There are no more discs here.
          while (frisbees_.size() > 0) {
            LOG(ERROR, "Dropping an extra disc since it can't exist\n");
            LOG(ERROR, "Upper is [%f %f]\n",
                upper_open_region_.upper_bound(),
                upper_open_region_.lower_bound());
            LOG(ERROR, "Lower is [%f %f]\n",
                lower_open_region_.upper_bound(),
                lower_open_region_.lower_bound());
            frisbees_.pop_back();
            --hopper_disc_count_;
            --total_disc_count_;
          }
          if (hopper_disc_count_ != 0) {
            LOG(ERROR,
                "Emptied the hopper out but there are still %" PRId32 " discs there\n",
                hopper_disc_count_);
            hopper_disc_count_ = 0;
          }
          hopper_clear_ = true;
        }
      }

      LOG(DEBUG, "READY_SHOOTER or SHOOT\n");
      break;
  }

  // Wait for a period of time to make sure that the disc gets sucked
  // in properly.  We need to do this regardless of what the indexer is doing.
  for (auto frisbee = frisbees_.begin();
      frisbee != frisbees_.end(); ++frisbee) {
    if (now - frisbee->bottom_negedge_time_ < kTransferOffDelay) {
      transfer_voltage = 12.0;
    }
  }

  // If we have 4 discs, it is time to preload.
  if (safe_to_change_state && hopper_disc_count_ >= 4) {
    switch (safe_goal_) {
      case Goal::HOLD:
      case Goal::READY_LOWER:
      case Goal::INTAKE:
        safe_goal_ = Goal::READY_SHOOTER;
        safe_to_change_state = false;
        LOG(INFO, "We have %" PRId32 " discs, time to preload automatically\n",
            hopper_disc_count_);
        break;
      case Goal::READY_SHOOTER:
      case Goal::SHOOT:
      case Goal::REINITIALIZE:
        break;
    }
  }

  // The only way out of the loader is to shoot the disc.  The FSM can only go
  // forwards.
  switch (loader_state_) {
    case LoaderState::READY:
      LOG(DEBUG, "Loader READY\n");
      // Open and down, ready to accept a disc.
      loader_up_ = false;
      disc_clamped_ = false;
      disc_ejected_ = false;
      disk_stuck_in_loader_ = false;
      if (loader_goal_ == LoaderGoal::GRAB ||
          loader_goal_ == LoaderGoal::SHOOT_AND_RESET || goal->force_fire) {
        if (goal->force_fire) {
          LOG(INFO, "Told to force fire, moving on\n");
        } else if (loader_goal_ == LoaderGoal::GRAB) {
          LOG(INFO, "Told to GRAB, moving on\n");
        } else {
          LOG(INFO, "Told to SHOOT_AND_RESET, moving on\n");
        }
        loader_state_ = LoaderState::GRABBING;
        loader_countdown_ = kGrabbingDelay;
      } else {
        break;
      }
    case LoaderState::GRABBING:
      LOG(DEBUG, "Loader GRABBING %d\n", loader_countdown_);
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
      LOG(DEBUG, "Loader GRABBED\n");
      // Grabber closed.
      loader_up_ = false;
      disc_clamped_ = true;
      disc_ejected_ = false;
      if (loader_goal_ == LoaderGoal::SHOOT_AND_RESET || goal->force_fire) {
        if (shooter.status.FetchLatest() || shooter.status.get()) {
          // TODO(aschuh): If we aren't shooting nicely, wait until the shooter
          // is up to speed rather than just spinning.
          if (shooter.status->average_velocity > 130 && shooter.status->ready) {
            loader_state_ = LoaderState::LIFTING;
            loader_countdown_ = kLiftingDelay;
            loader_timeout_ = 0;
            LOG(INFO, "Told to SHOOT_AND_RESET, moving on\n");
          } else {
            LOG(WARNING, "Told to SHOOT_AND_RESET, shooter too slow at %f\n",
                shooter.status->average_velocity);
            break;
          }
        } else {
          LOG(ERROR, "Told to SHOOT_AND_RESET, no shooter data, moving on.\n");
          loader_state_ = LoaderState::LIFTING;
          loader_countdown_ = kLiftingDelay;
          loader_timeout_ = 0;
        }
      } else if (loader_goal_ == LoaderGoal::READY) {
        LOG(ERROR, "Can't go to ready when we have something grabbed.\n");
        break;
      } else {
        break;
      }
    case LoaderState::LIFTING:
      LOG(DEBUG, "Loader LIFTING %d %d\n", loader_countdown_, loader_timeout_);
      // Lifting the disc.
      loader_up_ = true;
      disc_clamped_ = true;
      disc_ejected_ = false;
      if (position->loader_top) {
        if (loader_countdown_ > 0) {
          --loader_countdown_;
          loader_timeout_ = 0;
          break;
        } else {
          loader_state_ = LoaderState::LIFTED;
        }
      } else {
        // Restart the countdown if it bounces back down or whatever.
        loader_countdown_ = kLiftingDelay;
        ++loader_timeout_;
        if (loader_timeout_ > kLiftingTimeout) {
          LOG(ERROR, "Loader timeout while LIFTING %d\n", loader_timeout_);
          loader_state_ = LoaderState::LOWERING;
          loader_countdown_ = kLoweringDelay;
          loader_timeout_ = 0;
          disk_stuck_in_loader_ = true;
        } else {
          break;
        }
      }
    case LoaderState::LIFTED:
      LOG(DEBUG, "Loader LIFTED\n");
      // Disc lifted.  Time to eject it out.
      loader_up_ = true;
      disc_clamped_ = true;
      disc_ejected_ = false;
      loader_state_ = LoaderState::SHOOTING;
      loader_countdown_ = kShootingDelay;
    case LoaderState::SHOOTING:
      LOG(DEBUG, "Loader SHOOTING %d\n", loader_countdown_);
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
      LOG(DEBUG, "Loader SHOOT\n");
      // The disc has been shot.
      loader_up_ = true;
      disc_clamped_ = false;
      disc_ejected_ = true;
      loader_state_ = LoaderState::LOWERING;
      loader_countdown_ = kLoweringDelay;
      loader_timeout_ = 0;
    case LoaderState::LOWERING:
      LOG(DEBUG, "Loader LOWERING %d %d\n", loader_countdown_, loader_timeout_);
      // Lowering the loader back down.
      loader_up_ = false;
      disc_clamped_ = false;
      // We don't want to eject if we're stuck because it will force the disc
      // into the green loader wheel.
      disc_ejected_ = disk_stuck_in_loader_ ? false : true;
      if (position->loader_bottom) {
        if (loader_countdown_ > 0) {
          --loader_countdown_;
          loader_timeout_ = 0;
          break;
        } else {
          loader_state_ = LoaderState::LOWERED;
          --hopper_disc_count_;
          ++shot_disc_count_;
        }
      } else {
        // Restart the countdown if it bounces back up or something.
        loader_countdown_ = kLoweringDelay;
        ++loader_timeout_;
        if (loader_timeout_ > kLoweringTimeout) {
          LOG(ERROR, "Loader timeout while LOWERING %d\n", loader_timeout_);
          loader_state_ = LoaderState::LOWERED;
          disk_stuck_in_loader_ = true;
        } else {
          break;
        }
      }
    case LoaderState::LOWERED:
      LOG(DEBUG, "Loader LOWERED\n");
      loader_up_ = false;
      disc_ejected_ = false;
      is_shooting_ = false;
      if (disk_stuck_in_loader_) {
        disk_stuck_in_loader_ = false;
        disc_clamped_ = true;
        loader_state_ = LoaderState::GRABBED;
      } else {
        disc_clamped_ = false;
        loader_state_ = LoaderState::READY;
        // Once we have shot, we need to hang out in READY until otherwise
        // notified.
        loader_goal_ = LoaderGoal::READY;
      }
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
    last_top_disc_negedge_count_ = position->top_disc_negedge_count;
  }

  // Clear everything if we are supposed to re-initialize.
  if (goal_enum == Goal::REINITIALIZE) {
    safe_goal_ = Goal::REINITIALIZE;
    no_prior_position_ = true;
    hopper_disc_count_ = 0;
    total_disc_count_ = 0;
    shot_disc_count_ = 0;
    loader_state_ = LoaderState::READY;
    loader_goal_ = LoaderGoal::READY;
    loader_countdown_ = 0;
    loader_up_ = false;
    disc_clamped_ = false;
    disc_ejected_ = false;

    intake_voltage = 0.0;
    transfer_voltage = 0.0;
    wrist_loop_->U(0, 0) = 0.0;
    frisbees_.clear();
  }

  status->hopper_disc_count = hopper_disc_count_;
  status->total_disc_count = total_disc_count_;
  status->shot_disc_count = shot_disc_count_;
  status->preloaded = (loader_state_ != LoaderState::READY);
  status->is_shooting = is_shooting_;
  status->hopper_clear = hopper_clear_;

  if (output) {
    output->intake_voltage = intake_voltage;
    if (goal->override_transfer) {
      output->transfer_voltage = goal->transfer_voltage;
    } else {
      output->transfer_voltage = transfer_voltage;
    }
    if (goal->override_index) {
      output->index_voltage = goal->index_voltage;
    } else {
      output->index_voltage = wrist_loop_->U(0, 0);
    }
    output->loader_up = loader_up_;
    output->disc_clamped = disc_clamped_;
    output->disc_ejected = disc_ejected_;
  }

  if (safe_to_change_state) {
    safe_goal_ = goal_enum;
  }
  if (hopper_disc_count_ < 0) {
    LOG(ERROR, "NEGATIVE DISCS.  VERY VERY BAD\n");
  }
}

}  // namespace control_loops
}  // namespace frc971
