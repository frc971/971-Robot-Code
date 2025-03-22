#include <cmath>
#include <filesystem>
#include <regex>

#include "absl/flags/flag.h"
#include "absl/strings/str_format.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/vision/intrinsics_calibration_lib.h"
#include "frc971/vision/vision_generated.h"

// TODO: Would be nice to remove this, but it depends on year-by-year Constants
ABSL_FLAG(std::string, base_intrinsics, "",
          "Intrinsics to use for estimating board pose prior to solving "
          "for the new intrinsics.");
ABSL_FLAG(std::string, calibration_folder, "/tmp",
          "Folder to place calibration files.");
ABSL_FLAG(std::string, camera_id, "",
          "Camera ID in format YY-NN-- year and number.");
ABSL_FLAG(std::string, channel, "/camera", "Camera channel to use");
ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");
ABSL_FLAG(std::string, cpu_name, "", "Pi/Orin name to calibrate.");
ABSL_FLAG(bool, display_undistorted, false,
          "If true, display the undistorted image.");
ABSL_FLAG(bool, use_orin1, true,
          "Use only the orin1 node instead of the imu node.");

namespace frc971::vision {
namespace {

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  std::string hostname = absl::GetFlag(FLAGS_cpu_name);
  if (hostname == "") {
    hostname = aos::network::GetHostname();
    LOG(INFO) << "Using pi/orin name from hostname as " << hostname;
  }
  CHECK(!absl::GetFlag(FLAGS_base_intrinsics).empty())
      << "Need a base intrinsics json to use to auto-capture images when the "
         "camera moves.";
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();

  CHECK(aos::network::ParsePiOrOrin(hostname))
      << "Failed to parse node type from " << hostname
      << ".  Should be of form orin-971-1";
  CHECK(aos::network::ParsePiOrOrinNumber(hostname))
      << "Failed to parse node number from " << hostname
      << ".  Should be of form orin-7971-2";

  std::string camera_name = absl::StrCat(
      "/", aos::network::ParsePiOrOrin(hostname).value(),
      std::to_string(aos::network::ParsePiOrOrinNumber(hostname).value()),
      absl::GetFlag(FLAGS_channel));
  // THIS IS A HACK FOR 2024, since we call Orin2 "Imu"
  if (aos::network::ParsePiOrOrin(hostname).value() == "orin" &&
      aos::network::ParsePiOrOrinNumber(hostname).value() == 2) {
    LOG(INFO) << "\nHACK for 2024: Renaming orin2 to imu\n";
    camera_name =
        absl::StrCat(absl::GetFlag(FLAGS_use_orin1) ? "/orin1" : "/imu",
                     absl::GetFlag(FLAGS_channel));
  }
  CHECK(event_loop.GetChannel<CameraImage>(camera_name) != nullptr)
      << " invalid camera name provided as '" << camera_name << "'";

  IntrinsicsCalibration calibrator(
      &event_loop, hostname, camera_name, absl::GetFlag(FLAGS_camera_id),
      absl::GetFlag(FLAGS_base_intrinsics),
      absl::GetFlag(FLAGS_display_undistorted),
      absl::GetFlag(FLAGS_calibration_folder), exit_handle.get());

  if (!absl::GetFlag(FLAGS_image_load_path).empty()) {
    calibrator.LoadImagesFromPath(absl::GetFlag(FLAGS_image_load_path));
  } else {
    event_loop.Run();
  }

  calibrator.MaybeCalibrate();
}

}  // namespace
}  // namespace frc971::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::Main();
}
