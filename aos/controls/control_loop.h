#ifndef AOS_CONTROL_LOOP_CONTROL_LOOP_H_
#define AOS_CONTROL_LOOP_CONTROL_LOOP_H_

#include <string.h>
#include <atomic>

#include "aos/events/event-loop.h"
#include "aos/events/shm-event-loop.h"
#include "aos/queue.h"
#include "aos/time/time.h"
#include "aos/type_traits/type_traits.h"
#include "aos/util/log_interval.h"

namespace aos {
namespace controls {

// Interface to describe runnable jobs.
class Runnable {
 public:
  virtual ~Runnable() {}
  // Runs forever.
  virtual void Run() = 0;
  // Does one quick piece of work and return.  Does _not_ block.
  virtual void Iterate() = 0;
};

// Control loops run this often, "starting" at time 0.
constexpr ::std::chrono::nanoseconds kLoopFrequency =
    ::std::chrono::milliseconds(5);

// Provides helper methods to assist in writing control loops.
// This template expects to be constructed with a queue group as an argument
// that has a goal, position, status, and output queue.
// It will then call the RunIteration method every cycle that it has enough
// valid data for the control loop to run.
template <class T>
class ControlLoop : public Runnable {
 public:
  // Create some convenient typedefs to reference the Goal, Position, Status,
  // and Output structures.
  typedef typename std::remove_reference<
      decltype(*(static_cast<T *>(NULL)->goal.MakeMessage().get()))>::type
        GoalType;
  typedef typename std::remove_reference<
      decltype(*(static_cast<T *>(NULL)->position.MakeMessage().get()))>::type
        PositionType;
  typedef typename std::remove_reference<
    decltype(*(static_cast<T *>(NULL)->status.MakeMessage().get()))>::type
      StatusType;
  typedef typename std::remove_reference<
    decltype(*(static_cast<T *>(NULL)->output.MakeMessage().get()))>::type
      OutputType;

  ControlLoop(T *control_loop)
      : event_loop_(new ::aos::ShmEventLoop()), control_loop_(control_loop) {
    output_sender_ = event_loop_->MakeSender<OutputType>(
        ::std::string(control_loop_->name()) + ".output");
    status_sender_ = event_loop_->MakeSender<StatusType>(
        ::std::string(control_loop_->name()) + ".status");
    goal_fetcher_ = event_loop_->MakeFetcher<GoalType>(
        ::std::string(control_loop_->name()) + ".goal");
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
  virtual void Zero(OutputType *output) { output->Zero(); }

  // Runs the loop forever.
  void Run() override;

  // Runs one cycle of the loop.
  void Iterate() override;

 protected:
  void IteratePosition(const PositionType &position);

  static void Quit(int /*signum*/) {
    run_ = false;
  }

  // Runs an iteration of the control loop.
  // goal is the last goal that was sent.  It might be any number of cycles old
  // or nullptr if we haven't ever received a goal.
  // position is the current position, or nullptr if we didn't get a position
  // this cycle.
  // output is the values to be sent to the motors.  This is nullptr if the
  // output is going to be ignored and set to 0.
  // status is the status of the control loop.
  // Both output and status should be filled in by the implementation.
  virtual void RunIteration(const GoalType *goal,
                            const PositionType *position,
                            OutputType *output,
                            StatusType *status) = 0;

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
  ::std::unique_ptr<EventLoop> event_loop_;
  T *control_loop_;

  ::aos::Sender<OutputType> output_sender_;
  ::aos::Sender<StatusType> status_sender_;
  ::aos::Fetcher<GoalType> goal_fetcher_;

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

  static ::std::atomic<bool> run_;
};

}  // namespace controls
}  // namespace aos

#include "aos/controls/control_loop-tmpl.h"  // IWYU pragma: export

#endif
