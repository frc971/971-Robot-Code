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

// TODO: Would be nice to remove this, but it depends on year-by-year Constants
DEFINE_string(base_intrinsics, "",
              "Intrinsics to use for estimating board pose prior to solving "
              "for the new intrinsics.");
DEFINE_string(calibration_folder, ".", "Folder to place calibration files.");
DEFINE_string(camera_id, "", "Camera ID in format YY-NN-- year and number.");
DEFINE_string(channel, "/camera", "Camera channel to use");
DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_string(cpu_name, "", "Pi/Orin name to calibrate.");
DEFINE_bool(display_undistorted, false,
            "If true, display the undistorted image.");

namespace frc971::vision {
namespace {

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  std::string hostname = FLAGS_cpu_name;
  if (hostname == "") {
    hostname = aos::network::GetHostname();
    LOG(INFO) << "Using pi/orin name from hostname as " << hostname;
  }
  CHECK(!FLAGS_base_intrinsics.empty())
      << "Need a base intrinsics json to use to auto-capture images when the "
         "camera moves.";
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();

  CHECK(aos::network::ParsePiOrOrin(hostname))
      << "Failed to parse node type from " << hostname
      << ".  Should be of form orin1-971-1";
  CHECK(aos::network::ParsePiOrOrinNumber(hostname))
      << "Failed to parse node number from " << hostname
      << ".  Should be of form orin2-7971-1";

  std::string camera_name = absl::StrCat(
      "/", aos::network::ParsePiOrOrin(hostname).value(),
      std::to_string(aos::network::ParsePiOrOrinNumber(hostname).value()),
      FLAGS_channel);
  // THIS IS A HACK FOR 2024, since we call Orin2 "Imu"
  if (aos::network::ParsePiOrOrin(hostname).value() == "orin" &&
      aos::network::ParsePiOrOrinNumber(hostname).value() == 2) {
    LOG(INFO) << "\nHACK for 2024: Renaming orin2 to imu\n";
    camera_name = absl::StrCat("/imu", FLAGS_channel);
  }

  IntrinsicsCalibration extractor(&event_loop, hostname, camera_name,
                                  FLAGS_camera_id, FLAGS_base_intrinsics,
                                  FLAGS_display_undistorted,
                                  FLAGS_calibration_folder, exit_handle.get());

  event_loop.Run();

  extractor.MaybeCalibrate();
}

}  // namespace
}  // namespace frc971::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::Main();
}
