#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "y2022/setpoint_generated.h"

DEFINE_double(catapult_position, 0.03, "Catapult shot position");
DEFINE_double(catapult_velocity, 18.0, "Catapult shot velocity");
DEFINE_double(turret, 0.0, "Turret setpoint");

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

using y2022::input::joysticks::Setpoint;

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  auto setpoint_sender = event_loop.MakeSender<Setpoint>("/superstructure");

  aos::Sender<Setpoint>::Builder builder = setpoint_sender.MakeBuilder();

  Setpoint::Builder setpoint_builder = builder.MakeBuilder<Setpoint>();

  setpoint_builder.add_catapult_position(FLAGS_catapult_position);
  setpoint_builder.add_catapult_velocity(FLAGS_catapult_velocity);
  setpoint_builder.add_turret(FLAGS_turret);
  builder.CheckOk(builder.Send(setpoint_builder.Finish()));

  return 0;
}
