#ifndef AOS_CONTROL_LOOP_CONTROL_LOOP_H_
#define AOS_CONTROL_LOOP_CONTROL_LOOP_H_

#include <string.h>
#include <atomic>

#include "aos/common/queue.h"
#include "aos/common/time.h"
#include "aos/common/type_traits.h"
#include "aos/common/util/log_interval.h"

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

class SerializableControlLoop : public Runnable {
 public:
  // Returns the size of all the data to be sent when serialized.
  virtual size_t SeralizedSize() = 0;
  // Serialize the current data.
  virtual void Serialize(char *buffer) const = 0;
  // Serialize zeroed data in case the data is out of date.
  virtual void SerializeZeroMessage(char *buffer) const = 0;
  // Deserialize data into the control loop.
  virtual void Deserialize(const char *buffer) = 0;
  // Unique identifier for the control loop.
  // Most likely the hash of the queue group.
  virtual uint32_t UniqueID() = 0;
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
class ControlLoop : public SerializableControlLoop {
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

  ControlLoop(T *control_loop) : control_loop_(control_loop) {}

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
  // a safe state (generally motors off). For some subclasses, this will be a
  // bit different (ie pistons).
  // The implementation here creates a new Output message, calls Zero() on it,
  // and then sends it.
  virtual void ZeroOutputs();

  // Sets the output to zero.
  // Over-ride if a value of zero is not "off" for this subsystem.
  virtual void Zero(OutputType *output) { output->Zero(); }

  // Runs the loop forever.
  void Run() override;

  // Runs one cycle of the loop.
  void Iterate() override;

  // Returns the name of the queue group.
  const char *name() { return control_loop_->name(); }

  // Methods to serialize all the data that should be sent over the network.
  size_t SeralizedSize() override { return control_loop_->goal->Size(); }
  void Serialize(char *buffer) const override {
    control_loop_->goal->Serialize(buffer);
  }
  void SerializeZeroMessage(char *buffer) const override {
    GoalType zero_goal;
    zero_goal.Zero();
    zero_goal.Serialize(buffer);
  }

  void Deserialize(const char *buffer) override {
    ScopedMessagePtr<GoalType> new_msg = control_loop_->goal.MakeMessage();
    new_msg->Deserialize(buffer);
    new_msg.Send();
  }

  uint32_t UniqueID() override { return control_loop_->hash(); }


 protected:
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

  T *queue_group() { return control_loop_; }
  const T *queue_group() const { return control_loop_; }

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
  T *control_loop_;

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

#include "aos/common/controls/control_loop-tmpl.h"  // IWYU pragma: export

#endif
