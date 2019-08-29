#ifndef AOS_CONTROL_LOOP_CONTROL_LOOP_H_
#define AOS_CONTROL_LOOP_CONTROL_LOOP_H_

#include <string.h>
#include <atomic>

#include "aos/events/event_loop.h"
#include "aos/robot_state/joystick_state_generated.h"
#include "aos/robot_state/robot_state_generated.h"
#include "aos/time/time.h"
#include "aos/type_traits/type_traits.h"
#include "aos/util/log_interval.h"

namespace aos {
namespace controls {

// Control loops run this often, "starting" at time 0.
constexpr ::std::chrono::nanoseconds kLoopFrequency =
    ::std::chrono::milliseconds(5);

// Provides helper methods to assist in writing control loops.
// It will then call the RunIteration method every cycle that it has enough
// valid data for the control loop to run.
template <class GoalType, class PositionType, class StatusType,
          class OutputType>
class ControlLoop {
 public:
  ControlLoop(EventLoop *event_loop, const ::std::string &name)
      : event_loop_(event_loop), name_(name) {
    output_sender_ = event_loop_->MakeSender<OutputType>(name_);
    status_sender_ = event_loop_->MakeSender<StatusType>(name_);
    goal_fetcher_ = event_loop_->MakeFetcher<GoalType>(name_);
    robot_state_fetcher_ = event_loop_->MakeFetcher<::aos::RobotState>("/aos");
    joystick_state_fetcher_ =
        event_loop_->MakeFetcher<::aos::JoystickState>("/aos");

    event_loop_->MakeWatcher(name_, [this](const PositionType &position) {
      this->IteratePosition(position);
    });
  }

  const ::aos::RobotState &robot_state() const { return *robot_state_fetcher_; }
  bool has_joystick_state() const { return joystick_state_fetcher_.get(); }
  const ::aos::JoystickState &joystick_state() const {
    return *joystick_state_fetcher_;
  }

  // Returns true if all the counters etc in the sensor data have been reset.
  // This will return true only a single time per reset.
  bool WasReset() {
    if (reset_) {
      reset_ = false;
      return true;
    } else {
      return false;
    }
  }

  // Constructs and sends a message on the output queue which sets everything to
  // a safe state.  Default is to set everything to zero.  Override Zero below
  // to change that behavior.
  void ZeroOutputs();

  // Sets the output to zero.
  // Override this if a value of zero (or false) is not "off" for this
  // subsystem.
  virtual flatbuffers::Offset<OutputType> Zero(
      typename ::aos::Sender<OutputType>::Builder *builder) {
    return builder->template MakeBuilder<OutputType>().Finish();
  }

 protected:
  // Runs one cycle of the loop.
  void IteratePosition(const PositionType &position);

  EventLoop *event_loop() { return event_loop_; }

  // Returns the position context.  This is only valid inside the RunIteration
  // method.
  const aos::Context &position_context() { return event_loop_->context(); }

  // Runs an iteration of the control loop.
  // goal is the last goal that was sent.  It might be any number of cycles old
  // or nullptr if we haven't ever received a goal.
  // position is the current position, or nullptr if we didn't get a position
  // this cycle.
  // output is the values to be sent to the motors.  This is nullptr if the
  // output is going to be ignored and set to 0.
  // status is the status of the control loop.
  // Both output and status should be filled in by the implementation.
  virtual void RunIteration(
      const GoalType *goal, const PositionType *position,
      typename ::aos::Sender<OutputType>::Builder *output,
      typename ::aos::Sender<StatusType>::Builder *status) = 0;

 private:
  static constexpr ::std::chrono::milliseconds kStaleLogInterval =
      ::std::chrono::milliseconds(100);
  // The amount of time after the last PWM pulse we consider motors enabled for.
  // 100ms is the result of using an oscilliscope to look at the input and
  // output of a Talon. The Info Sheet also lists 100ms for Talon SR, Talon SRX,
  // and Victor SP.
  static constexpr ::std::chrono::milliseconds kPwmDisableTime =
      ::std::chrono::milliseconds(100);

  // Pointer to the queue group
  EventLoop *event_loop_;
  ::std::string name_;

  ::aos::Sender<OutputType> output_sender_;
  ::aos::Sender<StatusType> status_sender_;
  ::aos::Fetcher<GoalType> goal_fetcher_;
  ::aos::Fetcher<::aos::RobotState> robot_state_fetcher_;
  ::aos::Fetcher<::aos::JoystickState> joystick_state_fetcher_;

  // Fetcher only to be used for the Iterate method.  If Iterate is called, Run
  // can't be called.
  bool has_iterate_fetcher_ = false;
  ::aos::Fetcher<PositionType> iterate_position_fetcher_;

  bool reset_ = false;
  int32_t sensor_reader_pid_ = 0;

  ::aos::monotonic_clock::time_point last_pwm_sent_ =
      ::aos::monotonic_clock::min_time;

  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
  SimpleLogInterval no_sensor_state_ =
      SimpleLogInterval(kStaleLogInterval, ERROR, "no sensor state");
  SimpleLogInterval motors_off_log_ =
      SimpleLogInterval(kStaleLogInterval, WARNING, "motors disabled");
  SimpleLogInterval no_goal_ =
      SimpleLogInterval(kStaleLogInterval, ERROR, "no goal");
};

}  // namespace controls
}  // namespace aos

#include "aos/controls/control_loop-tmpl.h"  // IWYU pragma: export

#endif
