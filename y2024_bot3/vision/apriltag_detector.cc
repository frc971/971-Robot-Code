
#include <string>

#include "absl/flags/flag.h"

#include "aos/init.h"
#include "frc971/orin/gpu_apriltag.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/vision/vision_util.h"

ABSL_FLAG(std::string, channel, "/camera", "Channel name");
ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

void GpuApriltagDetector() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc971::constants::WaitForConstants<y2024_bot3::Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  const frc971::constants::ConstantsFetcher<y2024_bot3::Constants>
      calibration_data(&event_loop);

  CHECK(absl::GetFlag(FLAGS_channel).length() == 8);
  int camera_id = std::stoi(absl::GetFlag(FLAGS_channel).substr(7, 1));
  const frc971::vision::calibration::CameraCalibration *calibration =
      y2024_bot3::vision::FindCameraCalibration(
          calibration_data.constants(),
          event_loop.node()->name()->string_view(), camera_id);

  frc971::apriltag::ApriltagDetector detector(
      &event_loop, absl::GetFlag(FLAGS_channel), calibration);

  // TODO(austin): Figure out our core pinning strategy.
  // event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({5}));

  LOG(INFO) << "Setting scheduler priority";
  struct sched_param param;
  param.sched_priority = 21;
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0);

  LOG(INFO) << "Running event loop";
  // TODO(austin): Pre-warm it...
  event_loop.Run();
}  // namespace frc971::apriltag

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  GpuApriltagDetector();

  return 0;
}
