#include <sys/resource.h>
#include <sys/time.h>

#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "frc971/input/joystick_state_generated.h"
#include "glog/logging.h"

DEFINE_string(config, "aos_config.json", "Config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);
  aos::ShmEventLoop event_loop(&config.message());

  aos::Sender<aos::JoystickState> sender(
      event_loop.MakeSender<aos::JoystickState>("/imu/aos"));

  event_loop.MakeWatcher(
      "/roborio/aos", [&](const aos::JoystickState &joystick_state) {
        auto builder = sender.MakeBuilder();
        flatbuffers::Offset<aos::JoystickState> state_fbs =
            aos::CopyFlatBuffer(&joystick_state, builder.fbb());
        builder.CheckOk(builder.Send(state_fbs));
      });

  event_loop.Run();
  return 0;
}
