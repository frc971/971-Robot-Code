#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2022/vision/camera_reader.h"

// config used to allow running camera_reader independently.  E.g.,
// bazel run //y2022/vision:camera_reader -- --config y2022/config.json
//   --override_hostname pi-7971-1  --ignore_timestamps true
DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_uint32(exposure, 5, "Exposure time, in 100us increments");

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
  v4l2_reader.SetExposure(FLAGS_exposure);

  CameraReader camera_reader(&event_loop, &calibration_data.message(),
                             &v4l2_reader);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace y2022

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::CameraReaderMain();
}
