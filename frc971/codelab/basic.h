#ifndef FRC971_CODELAB_BASIC_H_
#define FRC971_CODELAB_BASIC_H_

#include "aos/controls/control_loop.h"
#include "aos/time/time.h"

#include "frc971/codelab/basic_goal_generated.h"
#include "frc971/codelab/basic_output_generated.h"
#include "frc971/codelab/basic_position_generated.h"
#include "frc971/codelab/basic_status_generated.h"

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
//  There are 4 channels (goal, position, status, output).
//
//  2 channels are input channels: goal, position.
//  2 channels are output channels: output, status.
//
// ::aos::controls::ControlLoop is a helper class that takes
// all the channel types as template parameters and then calls
// RunIteration() whenever a Position message is received.
// It will pass in the Position message and most recent Goal message
// and provide Builders that the RunIteration method should use to
// construct and send output/status messages.
//
// The various basic_*.fbs files define the  Goal, Position, Status, and Output
// messages.
//
// In order to get the tests to pass, you'll need to fill out the RunIteration()
// implementation in basic.cc so that it uses the input goal/position to
// meaningfully populate the output/status messages. You can find descriptions
// of exactly what the fields of the messages mean by reading all the *.fbs
// files, and the tests below can be reviewed to help understand exactly what
// behavior is expected.
//
// Once you can get the tests to pass, follow the directions in the
// documentation/tutorials/submitting-code-for-a-review.md file for creating a
// code review of this change. We will not actually *submit* the change (since
// that  would remove the challenge for future students), but we will go through
// the code review process.
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
