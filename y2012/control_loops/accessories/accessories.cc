#include "y2012/control_loops/accessories/accessories.q.h"

#include "aos/init.h"
#include "aos/controls/control_loop.h"

namespace y2012 {
namespace control_loops {
namespace accessories {

class AccessoriesLoop : public ::aos::controls::ControlLoop<
                            ::y2012::control_loops::AccessoriesQueue> {
 public:
  explicit AccessoriesLoop(
      ::aos::EventLoop *event_loop,
      const ::std::string &name = ".y2012.control_loops.accessories_queue")
      : ::aos::controls::ControlLoop<::y2012::control_loops::AccessoriesQueue>(
            event_loop, name) {}

  void RunIteration(
      const ::y2012::control_loops::AccessoriesQueue::Message *goal,
      const ::aos::control_loops::Position * /*position*/,
      ::y2012::control_loops::AccessoriesQueue::Message *output,
      ::aos::control_loops::Status * /*status*/) override {
    if (output) {
      *output = *goal;
    }
  }
};

}  // namespace accessories
}  // namespace control_loops
}  // namespace y2012

int main() {
  ::aos::Init();
  ::aos::ShmEventLoop event_loop;
  ::y2012::control_loops::accessories::AccessoriesLoop accessories(&event_loop);
  accessories.Run();
  ::aos::Cleanup();
}
