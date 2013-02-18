#ifndef AOS_CONTROL_LOOP_CONTROL_LOOP_H_
#define AOS_CONTROL_LOOP_CONTROL_LOOP_H_

#include <cstring>

#include "aos/aos_core.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/type_traits.h"
#include "aos/common/queue.h"

namespace aos {
namespace control_loops {

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

// Provides helper methods to assist in writing control loops.
// This template expects to be constructed with a queue group as an argument
// that has a goal, position, status, and output queue.
// It will then call the RunIteration method every cycle that it has enough
// valid data for the control loop to run.
// If has_position is false, the control loop will always use NULL as the
// position and not check the queue.  This is used for "loops" that control
// motors open loop.
template <class T, bool has_position = true>
class ControlLoop : public SerializableControlLoop {
 public:
  // Maximum age of position packets before the loop will be disabled due to
  // invalid position data.
  static const int kPositionTimeoutMs = 100;
  // Maximum age of driver station packets before the loop will be disabled.
  static const int kDSPacketTimeoutMs = 100;

  ControlLoop(T *control_loop) : control_loop_(control_loop) {}

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

  // Constructs and sends a message on the output queue which will stop all the
  // motors.  Calls Zero to clear all the state.
  void ZeroOutputs();

  // Sets the output to zero.
  // Over-ride if a value of zero is not "off" for this subsystem.
  virtual void Zero(OutputType *output) { output->Zero(); }

  // Runs the loop forever.
  virtual void Run();

  // Runs one cycle of the loop.
  virtual void Iterate();

  // Returns the name of the queue group.
  const char *name() { return control_loop_->name(); }

  // Methods to serialize all the data that should be sent over the network.
  virtual size_t SeralizedSize() { return control_loop_->goal->Size(); }
  virtual void Serialize(char *buffer) const {
    control_loop_->goal->Serialize(buffer);
  }
  virtual void SerializeZeroMessage(char *buffer) const {
    GoalType zero_goal;
    zero_goal.Zero();
    zero_goal.Serialize(buffer);
  }

  virtual void Deserialize(const char *buffer) {
    ScopedMessagePtr<GoalType> new_msg = control_loop_->goal.MakeMessage();
    new_msg->Deserialize(buffer);
    new_msg.Send();
  }

  virtual uint32_t UniqueID() { return control_loop_->hash(); }

 protected:
  // Runs an iteration of the control loop.
  // goal is the last goal that was sent.  It might be any number of cycles old.
  // position is the current position, or NULL if we didn't get a position this
  // cycle.
  // output is the values to be sent to the motors.  This is NULL if the output
  // is going to be ignored and set to 0.
  // status is the status of the control loop.
  // Both output and status should be filled in by the implementation.
  virtual void RunIteration(const GoalType *goal,
                            const PositionType *position,
                            OutputType *output,
                            StatusType *status) = 0;

 private:
  // Pointer to the queue group
  T *control_loop_;
};

}  // namespace control_loops
}  // namespace aos

#include "aos/common/control_loop/ControlLoop-tmpl.h"  // IWYU pragma: export

#endif
