#include "y2012/control_loops/accessories/accessories_generated.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/controls/control_loop.h"
#include "aos/controls/control_loops_generated.h"

namespace y2012 {
namespace control_loops {
namespace accessories {

class AccessoriesLoop : public ::aos::controls::ControlLoop<
                            Message, ::aos::control_loops::Position,
                            ::aos::control_loops::Status, Message> {
 public:
  explicit AccessoriesLoop(
      ::aos::EventLoop *event_loop,
      const ::std::string &name = ".y2012.control_loops.accessories_queue")
      : ::aos::controls::ControlLoop<Message, ::aos::control_loops::Position,
                                     ::aos::control_loops::Status, Message>(
            event_loop, name) {}

  void RunIteration(
      const Message *goal,
      const ::aos::control_loops::Position * /*position*/,
      ::aos::Sender<Message>::Builder *output,
      ::aos::Sender<::aos::control_loops::Status>::Builder * /*status*/) override {
    if (output) {
      //*output = *goal;
      Message::Builder output_builder = output->MakeBuilder<Message>();
      flatbuffers::Offset<flatbuffers::Vector<uint8_t>> solenoid_offset =
          output->fbb()->template CreateVector<uint8_t>(
              goal->solenoids()->data(), 3);
      output_builder.add_solenoids(solenoid_offset);
      flatbuffers::Offset<flatbuffers::Vector<double>> stick_offset =
          output->fbb()->template CreateVector<double>(
              goal->sticks()->data(), 2);
      output_builder.add_sticks(stick_offset);

      output_builder.Finish();
    }
  }
};

}  // namespace accessories
}  // namespace control_loops
}  // namespace y2012

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2012::control_loops::accessories::AccessoriesLoop accessories(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
