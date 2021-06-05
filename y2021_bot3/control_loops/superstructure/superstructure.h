#ifndef Y2021_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2021_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/controls/control_loop.h"
#include "aos/events/event_loop.h"
#include "y2021_bot3/constants.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_output_generated.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_position_generated.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_status_generated.h"

namespace y2021_bot3 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2021_bot3

#endif  // Y2021_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
