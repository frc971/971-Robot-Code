#include <cmath>
#include <regex>

#include "absl/strings/str_format.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/vision/intrinsics_calibration_lib.h"

DEFINE_string(calibration_folder, ".", "Folder to place calibration files.");
DEFINE_string(camera_id, "", "Camera ID in format YY-NN-- year and number.");
DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_bool(display_undistorted, false,
            "If true, display the undistorted image.");
DEFINE_string(pi, "", "Pi name to calibrate.");
DEFINE_string(base_intrinsics, "",
              "Intrinsics to use for estimating board pose prior to solving "
              "for the new intrinsics.");

namespace frc971 {
namespace vision {
namespace {

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  std::string hostname = FLAGS_pi;
  if (hostname == "") {
    hostname = aos::network::GetHostname();
    LOG(INFO) << "Using pi name from hostname as " << hostname;
  }
  CHECK(!FLAGS_base_intrinsics.empty())
      << "Need a base intrinsics json to use to auto-capture images when the "
         "camera moves.";
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();
  IntrinsicsCalibration extractor(
      &event_loop, hostname, FLAGS_camera_id, FLAGS_base_intrinsics,
      FLAGS_display_undistorted, FLAGS_calibration_folder, exit_handle.get());

  event_loop.Run();

  extractor.MaybeCalibrate();
}

}  // namespace
}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::Main();
}
