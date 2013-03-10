#ifndef FRC971_CONTROL_LOOPS_WRIST_H_
#define FRC971_CONTROL_LOOPS_WRIST_H_

#include <deque>
#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "aos/common/time.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/index/index_motor_plant.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class IndexTest_InvalidStateTest_Test;
class IndexTest_LostDisc_Test;
}

// This class represents a region of space.
class Region {
 public:
  Region () : upper_bound_(0.0), lower_bound_(0.0) {}

  // Restarts the region tracking by starting over with a 0 width region with
  // the bounds at [edge, edge].
  void Restart(double edge) {
    upper_bound_ = edge;
    lower_bound_ = edge;
  }

  // Expands the region to include the new point.
  void Expand(double new_point) {
    if (new_point > upper_bound_) {
      upper_bound_ = new_point;
    } else if (new_point < lower_bound_) {
      lower_bound_ = new_point;
    }
  }

  // Returns the width of the region.
  double width() const { return upper_bound_ - lower_bound_; }
  // Returns the upper and lower bounds.
  double upper_bound() const { return upper_bound_; }
  double lower_bound() const { return lower_bound_; }

 private:
  // Upper bound of the region.
  double upper_bound_;
  // Lower bound of the region.
  double lower_bound_;
};

class IndexMotor
    : public aos::control_loops::ControlLoop<control_loops::IndexLoop> {
 public:
  explicit IndexMotor(
      control_loops::IndexLoop *my_index = &control_loops::index_loop);

  static const double kTransferStartPosition;
  static const double kIndexStartPosition;
  // The distance from where the disc first grabs on the indexer to where it
  // just bairly clears the loader.
  static const double kIndexFreeLength;
  // The distance to where the disc just starts to enter the loader.
  static const double kLoaderFreeStopPosition;
  // The distance to where the next disc gets positioned while the current disc
  // is shooting.
  static const double kReadyToPreload;

  // Distance that the grabber pulls the disc in by.
  static const double kGrabberLength;
  // Distance to where the grabber takes over.
  static const double kGrabberStartPosition;

  // The distance to where the disc hits the back of the loader and is ready to
  // lift.
  static const double kReadyToLiftPosition;

  static const double kGrabberMovementVelocity;
  // TODO(aschuh): This depends on the shooter angle...
  // Distance to where the shooter is up and ready to shoot.
  static const double kLifterStopPosition;
  static const double kLifterMovementVelocity;

  // Distance to where the disc has been launched.
  // TODO(aschuh): This depends on the shooter angle...
  static const double kEjectorStopPosition;
  static const double kEjectorMovementVelocity;

  // Start and stop position of the bottom disc detect sensor in meters.
  static const double kBottomDiscDetectStart;
  static const double kBottomDiscDetectStop;
  // Delay between the negedge of the disc detect and when it engages on the
  // indexer.
  static const double kBottomDiscIndexDelay;

  static const double kTopDiscDetectStart;
  static const double kTopDiscDetectStop;

  // Minimum distance between 2 frisbees as seen by the top disc detect sensor.
  static const double kTopDiscDetectMinSeperation;

  // Converts the angle of the indexer to the angle of the disc.
  static double ConvertIndexToDiscAngle(const double angle);
  // Converts the angle of the indexer to the position that the center of the
  // disc has traveled.
  static double ConvertIndexToDiscPosition(const double angle);

  // Converts the angle of the transfer roller to the position that the center
  // of the disc has traveled.
  static double ConvertTransferToDiscPosition(const double angle);

  // Converts the distance around the indexer to the position of
  // the index roller.
  static double ConvertDiscPositionToIndex(const double position);
  // Converts the angle around the indexer to the position of the index roller.
  static double ConvertDiscAngleToIndex(const double angle);
  // Converts the angle around the indexer to the position of the disc in the
  // indexer.
  static double ConvertDiscAngleToDiscPosition(const double angle);
  // Converts the distance around the indexer to the angle of the disc around
  // the indexer.
  static double ConvertDiscPositionToDiscAngle(const double position);

  // Disc radius in meters.
  static const double kDiscRadius;
  // Roller radius in meters.
  static const double kRollerRadius;
  // Transfer roller radius in meters.
  static const double kTransferRollerRadius;

  // Time that it takes to grab the disc in cycles.
  static const int kGrabbingDelay;
  // Time that it takes to lift the loader in cycles.
  static const int kLiftingDelay;
  // Time that it takes to shoot the disc in cycles.
  static const int kShootingDelay;
  // Time that it takes to lower the loader in cycles.
  static const int kLoweringDelay;

  // Object representing a Frisbee tracked by the indexer.
  class Frisbee {
   public:
    Frisbee()
        : bottom_posedge_time_(0, 0),
          bottom_negedge_time_(0, 0) {
      Reset();
    }

    // Resets a Frisbee so it can be reused.
    void Reset() {
      bottom_posedge_time_ = ::aos::time::Time(0, 0);
      bottom_negedge_time_ = ::aos::time::Time(0, 0);
      has_been_indexed_ = false;
      index_start_position_ = 0.0;
    }

    // Returns true if the position is valid.
    bool has_position() const {
      return has_been_indexed_;
    }

    // Returns the most up to date and accurate position that we have for the
    // disc.  This is the indexer position that the disc grabbed at.
    double position() const {
      return index_start_position_;
    }

    // Returns the absolute position of the disc in meters in the hopper given
    // that the indexer is at the provided position.
    double absolute_position(const double index_position) const {
      return IndexMotor::ConvertIndexToDiscPosition(
          index_position - index_start_position_) +
          IndexMotor::kIndexStartPosition;
    }

    // Shifts the disc down the indexer by the provided offset.  This is to
    // handle when the cRIO reboots.
    void OffsetDisc(double offset) {
      index_start_position_ += offset;
    }

    // Potentially offsets the position with the knowledge that no discs are
    // currently blocking the top sensor.  This knowledge can be used to move
    // this disc if it is believed to be blocking the top sensor.
    // Returns the amount that the disc moved due to this observation.
    double ObserveNoTopDiscSensor(double index_position, double index_velocity);

    // Posedge and negedge disc times.
    ::aos::time::Time bottom_posedge_time_;
    ::aos::time::Time bottom_negedge_time_;

    // True if the disc has a valid index position.
    bool has_been_indexed_;
    // Location of the index when the disc first contacted it.
    double index_start_position_;
  };

  // Returns where the indexer thinks the frisbees are.
  const ::std::deque<Frisbee> &frisbees() const { return frisbees_; }

 protected:
  virtual void RunIteration(
      const control_loops::IndexLoop::Goal *goal,
      const control_loops::IndexLoop::Position *position,
      control_loops::IndexLoop::Output *output,
      control_loops::IndexLoop::Status *status);

 private:
  friend class testing::IndexTest_InvalidStateTest_Test;
  friend class testing::IndexTest_LostDisc_Test;

  // This class implements the CapU function correctly given all the extra
  // information that we know about from the wrist motor.
  class IndexStateFeedbackLoop : public StateFeedbackLoop<2, 1, 1> {
   public:
    IndexStateFeedbackLoop(StateFeedbackLoop<2, 1, 1> loop)
        : StateFeedbackLoop<2, 1, 1>(loop),
          low_voltage_count_(0) {
    }

    // Voltage below which the indexer won't move with a disc in it.
    static const double kMinMotionVoltage;
    // Maximum number of cycles to apply a low voltage to the motor.
    static const double kNoMotionCuttoffCount;

    // Caps U, but disables the motor after a number of cycles of inactivity.
    virtual void CapU();
   private:
    // Number of cycles that we have seen a small voltage being applied.
    uint32_t low_voltage_count_;
  };

  // Sets disc_position to the minimum or maximum disc position.
  // Sets found_disc to point to the frisbee that was found, and ignores it if
  // found_disc is NULL.
  // Returns true if there were discs, and false if there weren't.
  // On false, disc_position is left unmodified.
  bool MinDiscPosition(double *disc_position, Frisbee **found_disc);
  bool MaxDiscPosition(double *disc_position, Frisbee **found_disc);

  // The state feedback control loop to talk to for the index.
  ::std::unique_ptr<IndexStateFeedbackLoop> wrist_loop_;

  // Count of the number of discs that we have collected.
  int32_t hopper_disc_count_;
  int32_t total_disc_count_;

  enum class Goal {
    // Hold position, in a low power state.
    HOLD = 0,
    // Get ready to load discs by shifting the discs down.
    READY_LOWER = 1,
    // Ready the discs, spin up the transfer roller, and accept discs.
    INTAKE = 2,
    // Get ready to shoot, and place a disc in the loader.
    READY_SHOOTER = 3,
    // Shoot at will.
    SHOOT = 4
  };

  // These two enums command and track the loader loading discs into the
  // shooter.
  enum class LoaderState {
    // Open and down, ready to accept a disc.
    READY,
    // Closing the grabber.
    GRABBING,
    // Grabber closed.
    GRABBED,
    // Lifting the disc.
    LIFTING,
    // Disc lifted.
    LIFTED,
    // Ejecting the disc into the shooter.
    SHOOTING,
    // The disc has been shot.
    SHOOT,
    // Lowering the loader back down.
    LOWERING,
    // The indexer is lowered.
    LOWERED
  };

  // TODO(aschuh): If we are grabbed and asked to be ready, now what?
  // LOG ?
  enum class LoaderGoal {
    // Get the loader ready to accept another disc.
    READY,
    // Grab a disc now.
    GRAB,
    // Lift it up, shoot, and reset.
    // Waits to shoot until the shooter is stable.
    // Resets the goal to READY once one disc has been shot.
    SHOOT_AND_RESET
  };

  // The current goal
  Goal safe_goal_;

  // Loader goal, state, and counter.
  LoaderGoal loader_goal_;
  LoaderState loader_state_;
  int loader_countdown_;

  // Current state of the pistons.
  bool loader_up_;
  bool disc_clamped_;
  bool disc_ejected_;

  // The frisbee that is flying through the transfer rollers.
  Frisbee transfer_frisbee_;

  // Bottom disc detect from the last valid packet for detecting edges.
  bool last_bottom_disc_detect_;
  bool last_top_disc_detect_;
  int32_t last_bottom_disc_posedge_count_;
  int32_t last_bottom_disc_negedge_count_;
  int32_t last_bottom_disc_negedge_wait_count_;
  int32_t last_top_disc_posedge_count_;
  int32_t last_top_disc_negedge_count_;

  // Frisbees are in order such that the newest frisbee is on the front.
  ::std::deque<Frisbee> frisbees_;

  // True if we haven't seen a position before.
  bool no_prior_position_;
  // Number of position messages that we have missed in a row.
  uint32_t missing_position_count_;

  // The no-disc regions for both the bottom and top beam break sensors.
  Region upper_open_region_;
  Region lower_open_region_;

  DISALLOW_COPY_AND_ASSIGN(IndexMotor);
};
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_WRIST_H_
