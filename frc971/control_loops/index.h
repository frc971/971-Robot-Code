#ifndef FRC971_CONTROL_LOOPS_WRIST_H_
#define FRC971_CONTROL_LOOPS_WRIST_H_

#include <memory>
#include <deque>

#include "aos/common/control_loop/ControlLoop.h"
#include "aos/common/time.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/index_motor.q.h"
#include "frc971/control_loops/index_motor_plant.h"

namespace frc971 {
namespace control_loops {

class IndexMotor
    : public aos::control_loops::ControlLoop<control_loops::IndexLoop> {
 public:
  explicit IndexMotor(
      control_loops::IndexLoop *my_index = &control_loops::index);

  // Converts the angle of the indexer to the angle of the disc.
  static double ConvertIndexToDiscAngle(const double angle);
  // Converts the angle of the indexer to the position that the center of the
  // disc has traveled.
  static double ConvertIndexToDiscPosition(const double angle);

  // Converts the angle around the indexer to the position of the index roller.
  static double ConvertDiscAngleToIndex(const double angle);
  // Converts the angle around the indexer to the position of the disc in the
  // indexer.
  static double ConvertDiscAngleToDiscPosition(const double angle);

  // Disc radius in meters.
  const static double kDiscRadius;
  // Roller radius in meters.
  const static double kRollerRadius;

  class Frisbee {
   public:
    Frisbee()
        : bottom_posedge_time_(0, 0),
          bottom_negedge_time_(0, 0),
          index_start_time_(0, 0) {
      Reset();
    }

    void Reset() {
      bottom_posedge_time_ = ::aos::time::Time(0, 0);
      bottom_negedge_time_ = ::aos::time::Time(0, 0);
      index_start_time_ = ::aos::time::Time(0, 0);
      has_been_indexed_ = false;
      index_start_position_ = 0.0;
    }

    ::aos::time::Time bottom_posedge_time_;
    ::aos::time::Time bottom_negedge_time_;
    ::aos::time::Time index_start_time_;
    bool has_been_indexed_;
    double index_start_position_;
  };

 protected:
  virtual void RunIteration(
      const control_loops::IndexLoop::Goal *goal,
      const control_loops::IndexLoop::Position *position,
      control_loops::IndexLoop::Output *output,
      control_loops::IndexLoop::Status *status);

 private:
  // Fetches and locally caches the latest set of constants.
  bool FetchConstants();

  // The state feedback control loop to talk to for the index.
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> wrist_loop_;

  // Local cache of the index geometry constants.
  double horizontal_lower_limit_;
  double horizontal_upper_limit_;
  double horizontal_hall_effect_start_angle_;
  double horizontal_zeroing_speed_;

  // Count of the number of discs that we have collected.
  uint32_t hopper_disc_count_;
  uint32_t total_disc_count_;

  enum Goal {
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

  // The current goal
  Goal safe_goal_;

  // Current state of the pistons.
  bool loader_up_;
  bool disc_clamped_;
  bool disc_ejected_;

  //::aos::time::Time disc_bottom_posedge_time_;
  //::aos::time::Time disc_bottom_negedge_time_;
  // The frisbee that is flying through the transfer rollers.
  Frisbee transfer_frisbee_;

  bool last_bottom_disc_detect_;

  // Frisbees are in order such that the newest frisbee is on the front.
  ::std::deque<Frisbee> frisbees_;
  // std::array ?

  DISALLOW_COPY_AND_ASSIGN(IndexMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_WRIST_H_
