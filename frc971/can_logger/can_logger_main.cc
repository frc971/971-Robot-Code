#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/can_logger/can_logger.h"

ABSL_FLAG(std::string, interface_name, "can0", "Can interface to use");

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  frc971::can_logger::CanLogger can_logger(&event_loop, "/can",
                                           absl::GetFlag(FLAGS_interface_name));

  event_loop.Run();

  return 0;
}
