#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "gflags/gflags.h"
#include "y2020/setpoint_generated.h"

DEFINE_double(accelerator, 250.0, "Accelerator speed");
DEFINE_double(finisher, 500.0, "Finsher speed");
DEFINE_double(hood, 0.45, "Hood setpoint");

int main(int argc, char ** argv) {
  aos::InitGoogle(&argc, &argv);
  aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  aos::ShmEventLoop event_loop(&config.message());

  ::aos::Sender<y2020::joysticks::Setpoint> setpoint_sender =
      event_loop.MakeSender<y2020::joysticks::Setpoint>("/superstructure");

  aos::Sender<y2020::joysticks::Setpoint>::Builder builder =
      setpoint_sender.MakeBuilder();

  y2020::joysticks::Setpoint::Builder setpoint_builder =
      builder.MakeBuilder<y2020::joysticks::Setpoint>();

  setpoint_builder.add_accelerator(FLAGS_accelerator);
  setpoint_builder.add_finisher(FLAGS_finisher);
  setpoint_builder.add_hood(FLAGS_hood);
  builder.Send(setpoint_builder.Finish());

  return 0;
}
