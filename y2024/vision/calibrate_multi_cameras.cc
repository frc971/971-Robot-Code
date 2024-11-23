#include <numeric>

#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/vision/calibrate_multi_cameras_lib.h"
#include "frc971/vision/vision_util_lib.h"
#include "y2024/constants/simulated_constants_sender.h"
#include "y2024/vision/vision_util.h"

ABSL_FLAG(std::string, config, "",
          "If set, override the log's config file with this one.");
ABSL_FLAG(std::string, constants_path, "y2024/constants/constants.json",
          "Path to the constant file");
ABSL_FLAG(double, max_pose_error_ratio, 0.4,
          "Throw out target poses with a higher pose error ratio than this");
ABSL_FLAG(bool, robot, false,
          "If true we're calibrating extrinsics for the robot, use the "
          "correct node path for the robot.");

ABSL_DECLARE_FLAG(int32_t, min_target_id);
ABSL_DECLARE_FLAG(int32_t, max_target_id);
ABSL_DECLARE_FLAG(double, outlier_std_devs);

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  std::vector<frc971::vision::CameraNode> node_list(
      frc971::vision::CreateNodeList());

  std::map<std::string, int> ordering_map(
      frc971::vision::CreateOrderingMap(node_list));

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config =
      (absl::GetFlag(FLAGS_config).empty()
           ? std::nullopt
           : std::make_optional(
                 aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config))));

  // open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)),
      config.has_value() ? &config->message() : nullptr);

  reader.RemapLoggedChannel("/imu/constants", "y2024.Constants");
  reader.RemapLoggedChannel("/orin1/constants", "y2024.Constants");
  if (absl::GetFlag(FLAGS_robot)) {
    reader.RemapLoggedChannel("/roborio/constants", "y2024.Constants");
  }
  reader.Register();

  y2024::SendSimulationConstants(reader.event_loop_factory(),
                                 absl::GetFlag(FLAGS_team_number),
                                 absl::GetFlag(FLAGS_constants_path));

  auto find_calibration = [](aos::EventLoop *const event_loop,
                             std::string node_name, int camera_number)
      -> const frc971::vision::calibration::CameraCalibration * {
    frc971::constants::ConstantsFetcher<y2024::Constants> constants_fetcher(
        event_loop);
    // Get the calibration for this orin/camera pair
    return y2024::vision::FindCameraCalibration(constants_fetcher.constants(),
                                                node_name, camera_number);
  };

  frc971::vision::ExtrinsicsMain(node_list, find_calibration,
                                 y2024::vision::kOrinColors, &reader,
                                 ordering_map);
}
