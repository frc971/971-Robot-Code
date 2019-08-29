#ifndef FRC971_CODELAB_BASIC_H_
#define FRC971_CODELAB_BASIC_H_

#include "aos/controls/control_loop.h"
#include "aos/time/time.h"

#include "frc971/codelab/basic_generated.h"

namespace frc971 {
namespace codelab {

// This codelab helps build basic knowledge of how to use 971 control loop
// primatives.
//
// The meat of the task is to make the tests pass.
//
// Run the tests with:
//  $ bazel run //frc971/codelab:basic_test -- --gtest_color=yes
//
// Control loops all follow the same convention:
//  There are 4 queues (goal, position, status, output).
//
//  2 queues are input queues: goal, position.
//  2 queues are output queues: output, status.
//
// ::aos::controls::ControlLoop is a helper class that takes
// a queue_group type from a .h file, and organizes to call
// RunIteration() at a consistent interval. It will fetch from
// goal and position messages from the goal and position queue,
// and publish an output and status result to the output and status
// queues.
//
// The basic.q file will construct boilerplate c++ code for the
// Goal, Position, Status, Message
// types, and construct static variables for fetching these named queues.
// Inquisitive souls can check out:
//  $ bazel build //frc971/codelab:basic_queue
//  $ vim bazel-genfiles/frc971/codelab/basic.q.{cc,h} -o
//  from the 971-Robot-Code directory.
//
// Order of approaching this should be:
// - Read the BUILD file and learn about what code is being generated.
// - Read basic.q, and familiarize yourself on the inputs and types involved.
class Basic
    : public ::aos::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  explicit Basic(::aos::EventLoop *event_loop,
                 const ::std::string &name = "/codelab");

 protected:
  void RunIteration(const Goal *goal, const Position *position,
                    aos::Sender<Output>::Builder *output,
                    aos::Sender<Status>::Builder *status) override;
};

}  // namespace codelab
}  // namespace frc971

#endif  // FRC971_CODELAB_BASIC_H_
