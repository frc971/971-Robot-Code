#ifndef Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
#define Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_

#include <memory>

#include "aos/controls/control_loop.h"
#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2016/control_loops/shooter/shooter_goal_generated.h"
#include "y2016/control_loops/shooter/shooter_integral_plant.h"
#include "y2016/control_loops/shooter/shooter_output_generated.h"
#include "y2016/control_loops/shooter/shooter_position_generated.h"
#include "y2016/control_loops/shooter/shooter_status_generated.h"

namespace y2016 {
namespace control_loops {
namespace shooter {

namespace {
constexpr double kTolerance = 10.0;
}  // namespace

class ShooterSide {
 public:
  ShooterSide();

  // Sets the velocity goal in radians/sec
  void set_goal(double angular_velocity_goal);
  // Sets the current encoder position in radians
  void set_position(double current_position);

  // Populates the status structure.
  flatbuffers::Offset<ShooterSideStatus> SetStatus(
      flatbuffers::FlatBufferBuilder *fbb);

  // Returns the control loop calculated voltage.
  double voltage() const;

  // Returns the instantaneous velocity.
  double velocity() const { return loop_->X_hat(1, 0); }

  // Executes the control loop for a cycle.
  void Update(bool disabled);

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // The control loop.
  ::std::unique_ptr<StateFeedbackLoop<3, 1, 1>> loop_;

  // History array for calculating a filtered angular velocity.
  static constexpr int kHistoryLength = 10;
  ::std::array<double, kHistoryLength> history_;
  ptrdiff_t history_position_ = 0;

  DISALLOW_COPY_AND_ASSIGN(ShooterSide);
};

class Shooter
    : public ::aos::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  explicit Shooter(::aos::EventLoop *event_loop,
                   const ::std::string &name = "/shooter");

  enum class ShooterLatchState {
    // Any shoot commands will be passed through without modification.
    PASS_THROUGH = 0,
    // We are latched shooting waiting for the wheel to loose RPM.
    WAITING_FOR_SPINDOWN = 1,
    // We are latched shooting waiting for the wheel to spin back up.
    WAITING_FOR_SPINUP = 2,
    // Increment the shot count for the Status.
    INCREMENT_SHOT_COUNT = 3,
    // Wait until the button is released.
    WAITING_FOR_SHOT_NEGEDGE = 4,
  };

 protected:
  void RunIteration(const Goal *goal, const Position *position,
                    aos::Sender<Output>::Builder *output,
                    aos::Sender<Status>::Builder *status) override;

 private:
  ShooterSide left_, right_;

  // The number of shots since starting the Shooter.
  uint32_t shots_;

  // Current state.
  ShooterLatchState state_ = ShooterLatchState::PASS_THROUGH;
  ::aos::monotonic_clock::time_point last_pre_shot_timeout_;

  DISALLOW_COPY_AND_ASSIGN(Shooter);
};

}  // namespace shooter
}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
