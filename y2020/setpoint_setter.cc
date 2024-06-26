#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2020/setpoint_generated.h"

ABSL_FLAG(double, accelerator, 250.0, "Accelerator speed");
ABSL_FLAG(double, finisher, 500.0, "Finsher speed");
ABSL_FLAG(double, hood, 0.45, "Hood setpoint");
ABSL_FLAG(double, turret, 0.0, "Turret setpoint");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  aos::ShmEventLoop event_loop(&config.message());

  ::aos::Sender<y2020::joysticks::Setpoint> setpoint_sender =
      event_loop.MakeSender<y2020::joysticks::Setpoint>("/superstructure");

  aos::Sender<y2020::joysticks::Setpoint>::Builder builder =
      setpoint_sender.MakeBuilder();

  y2020::joysticks::Setpoint::Builder setpoint_builder =
      builder.MakeBuilder<y2020::joysticks::Setpoint>();

  setpoint_builder.add_accelerator(absl::GetFlag(FLAGS_accelerator));
  setpoint_builder.add_finisher(absl::GetFlag(FLAGS_finisher));
  setpoint_builder.add_hood(absl::GetFlag(FLAGS_hood));
  setpoint_builder.add_turret(absl::GetFlag(FLAGS_turret));
  builder.CheckOk(builder.Send(setpoint_builder.Finish()));

  return 0;
}
