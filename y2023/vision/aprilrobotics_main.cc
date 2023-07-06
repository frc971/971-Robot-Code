#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "y2023/vision/aprilrobotics.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

namespace y2023::vision {
void AprilViewerMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  frc971::constants::WaitForConstants<Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  AprilRoboticsDetector detector(&event_loop, "/camera");

  detector.SetWorkerpoolAffinities();

  event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({5}));

  struct sched_param param;
  param.sched_priority = 21;
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0);

  event_loop.Run();
}

}  // namespace y2023::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::AprilViewerMain();

  return 0;
}
