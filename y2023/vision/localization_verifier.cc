#include "glog/logging.h"

#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc971/vision/vision_generated.h"
#include "y2023/localizer/localizer.h"
#include "y2023/localizer/utils.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

DEFINE_uint64(min_april_id, 1,
              "Minimum april id to consider as part of the field and ignore");
DEFINE_uint64(max_april_id, 8,
              "Maximum april id to consider as part of the field and ignore");

// This binary allows us to place extra april tags on the field and verify
// that we can compute their field pose correctly

namespace y2023::vision {

using localizer::Localizer;

Localizer::Transform LocalizerOutputToTransform(
    const frc971::controls::LocalizerOutput &localizer) {
  const auto T_field_robot =
      Eigen::Translation3d(localizer.x(), localizer.y(), 0.0);

  Eigen::AngleAxisd robot_yaw_angle(localizer.theta(),
                                    Eigen::Vector3d::UnitZ());
  const auto R_field_robot = Eigen::Quaterniond(robot_yaw_angle);
  const Localizer::Transform H_field_robot =
      (T_field_robot * R_field_robot).matrix();
  return H_field_robot;
}

void HandleDetections(
    const frc971::vision::TargetMap &detections,
    const Localizer::Transform &H_robot_camera,
    aos::Fetcher<frc971::controls::LocalizerOutput> *localizer_fetcher) {
  localizer_fetcher->Fetch();
  CHECK(localizer_fetcher->get());

  for (const auto *target_pose : *detections.target_poses()) {
    // Only look at april tags not part of the field
    if (target_pose->id() >= FLAGS_min_april_id &&
        target_pose->id() <= FLAGS_max_april_id) {
      continue;
    }

    const Localizer::Transform H_camera_target =
        localizer::PoseToTransform(target_pose);
    const Localizer::Transform H_field_robot =
        LocalizerOutputToTransform(*localizer_fetcher->get());

    // Get the field-based target pose using the detection, extrinsics, and
    // localizer output
    const Localizer::Transform H_field_target =
        H_field_robot * H_robot_camera * H_camera_target;

    LOG(INFO) << "Field to target " << target_pose->id();
    LOG(INFO) << "H_field_robot " << H_field_robot;
    LOG(INFO) << "H_robot_camera " << H_robot_camera;
    LOG(INFO) << "H_camera_target " << H_camera_target;
    LOG(INFO) << "Transform: " << H_field_target;
    LOG(INFO) << "Translation: "
              << Eigen::Affine3d(H_field_target).translation();
    LOG(INFO);
  }
}

void LocalizationVerifierMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  frc971::constants::WaitForConstants<Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  frc971::constants::ConstantsFetcher<Constants> constants_fetcher(&event_loop);

  aos::Fetcher<frc971::controls::LocalizerOutput> localizer_fetcher =
      event_loop.MakeFetcher<frc971::controls::LocalizerOutput>("/localizer");

  for (const auto *camera : *constants_fetcher.constants().cameras()) {
    CHECK(camera->has_calibration());
    Localizer::Transform H_robot_camera =
        frc971::control_loops::drivetrain::FlatbufferToTransformationMatrix(
            *camera->calibration()->fixed_extrinsics());

    const std::string_view pi_name =
        camera->calibration()->node_name()->string_view();
    event_loop.MakeWatcher(
        absl::StrCat("/", pi_name, "/camera"),
        [H_robot_camera,
         &localizer_fetcher](const frc971::vision::TargetMap &target_map) {
          HandleDetections(target_map, H_robot_camera, &localizer_fetcher);
        });
  }
  event_loop.Run();
}

}  // namespace y2023::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::LocalizationVerifierMain();
}
