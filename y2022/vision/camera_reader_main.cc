#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2022/vision/camera_reader.h"

// config used to allow running camera_reader independently.  E.g.,
// bazel run //y2022/vision:camera_reader -- --config y2022/aos_config.json
//   --override_hostname pi-7971-1  --ignore_timestamps true
ABSL_DECLARE_FLAG(bool, use_outdoors);
ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");
ABSL_FLAG(double, duty_cycle, 0.65, "Duty cycle of the LEDs");
ABSL_FLAG(uint32_t, exposure, 3,
          "Exposure time, in 100us increments; 0 implies auto exposure");
ABSL_FLAG(uint32_t, outdoors_exposure, 2,
          "Exposure time when using --use_outdoors, in 100us increments; 0 "
          "implies auto exposure");

namespace y2022::vision {
namespace {

using namespace frc971::vision;

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  const aos::FlatbufferSpan<calibration::CalibrationData> calibration_data(
      CalibrationData());
  CHECK(calibration_data.Verify());

  aos::ShmEventLoop event_loop(&config.message());

  // First, log the data for future reference.
  {
    aos::Sender<calibration::CalibrationData> calibration_data_sender =
        event_loop.MakeSender<calibration::CalibrationData>("/camera");
    CHECK_EQ(calibration_data_sender.Send(calibration_data),
             aos::RawSender::Error::kOk);
  }

  V4L2Reader v4l2_reader(&event_loop, "/dev/video0");
  const uint32_t exposure = (absl::GetFlag(FLAGS_use_outdoors)
                                 ? absl::GetFlag(FLAGS_outdoors_exposure)
                                 : absl::GetFlag(FLAGS_exposure));
  if (exposure > 0) {
    LOG(INFO) << "Setting camera to Manual Exposure mode with exposure = "
              << exposure << " or " << static_cast<double>(exposure) / 10.0
              << " ms";
    v4l2_reader.SetExposure(exposure);
  } else {
    LOG(INFO) << "Setting camera to use Auto Exposure";
    v4l2_reader.UseAutoExposure();
  }

  CameraReader camera_reader(&event_loop, &calibration_data.message(),
                             &v4l2_reader);
  camera_reader.SetDutyCycle(absl::GetFlag(FLAGS_duty_cycle));

  event_loop.Run();
}

}  // namespace
}  // namespace y2022::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::CameraReaderMain();
}
