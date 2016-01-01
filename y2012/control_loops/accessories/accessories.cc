#include "y2012/control_loops/accessories/accessories.q.h"

#include "aos/linux_code/init.h"
#include "aos/common/controls/control_loop.h"

namespace y2012 {
namespace control_loops {
namespace accessories {

class AccessoriesLoop : public ::aos::controls::ControlLoop<
                            ::y2012::control_loops::AccessoriesQueue> {
 public:
  explicit AccessoriesLoop(
      ::y2012::control_loops::AccessoriesQueue *my_accessories =
          &::y2012::control_loops::accessories_queue)
      : ::aos::controls::ControlLoop<::y2012::control_loops::AccessoriesQueue>(
            my_accessories) {}

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
  ::y2012::control_loops::accessories::AccessoriesLoop accessories;
  accessories.Run();
  ::aos::Cleanup();
}
