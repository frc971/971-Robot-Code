#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2022/vision/camera_reader.h"

// config used to allow running camera_reader independently.  E.g.,
// bazel run //y2022/vision:camera_reader -- --config y2022/aos_config.json
//   --override_hostname pi-7971-1  --ignore_timestamps true
DECLARE_bool(use_outdoors);
DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_double(duty_cycle, 0.65, "Duty cycle of the LEDs");
DEFINE_uint32(exposure, 5,
              "Exposure time, in 100us increments; 0 implies auto exposure");
DEFINE_uint32(outdoors_exposure, 4,
              "Exposure time when using --use_outdoors, in 100us increments; 0 "
              "implies auto exposure");

namespace y2022 {
namespace vision {
namespace {

using namespace frc971::vision;

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

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
  const uint32_t exposure =
      (FLAGS_use_outdoors ? FLAGS_outdoors_exposure : FLAGS_exposure);
  if (exposure > 0) {
    v4l2_reader.SetExposure(exposure);
  }

  CameraReader camera_reader(&event_loop, &calibration_data.message(),
                             &v4l2_reader);
  camera_reader.SetDutyCycle(FLAGS_duty_cycle);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace y2022

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::CameraReaderMain();
}
