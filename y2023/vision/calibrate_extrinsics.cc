#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "absl/strings/str_format.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/extrinsics_calibration.h"
#include "frc971/vision/vision_generated.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2023/constants/constants_generated.h"
#include "y2023/vision/vision_util.h"

#include <opencv2/core/eigen.hpp>

DEFINE_string(pi, "pi-7971-2", "Pi name to calibrate.");
DEFINE_bool(plot, false, "Whether to plot the resulting data.");
DEFINE_string(target_type, "charuco_diamond",
              "Type of target: aruco|charuco|charuco_diamond");
DEFINE_string(image_channel, "/camera", "Channel to listen for images on");
DEFINE_string(output_logs, "/tmp/calibration/",
              "Output folder for visualization logs.");
DEFINE_string(base_calibration, "",
              "Intrinsics (and optionally extrinsics) to use for extrinsics "
              "calibration.");
DEFINE_string(output_calibration, "",
              "Output json file to use for the resulting calibration "
              "(intrinsics and extrinsics).");

namespace frc971 {
namespace vision {
namespace chrono = std::chrono;
using aos::distributed_clock;
using aos::monotonic_clock;

struct CalibrationAccumulatorState {
  CalibrationAccumulatorState(
      aos::SimulatedEventLoopFactory *factory, CalibrationData *data,
      aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> *intrinsics)
      : factory(factory), data(data), intrinsics(intrinsics) {}
  aos::SimulatedEventLoopFactory *factory;
  CalibrationData *data;
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> *intrinsics;
  std::unique_ptr<aos::EventLoop> imu_event_loop;
  std::unique_ptr<aos::EventLoop> pi_event_loop;
  std::unique_ptr<aos::EventLoop> logger_event_loop;
  std::unique_ptr<aos::logger::Logger> logger;
  std::unique_ptr<Calibration> extractor;
  void MaybeMakeExtractor() {
    if (imu_event_loop && pi_event_loop) {
      TargetType target_type = TargetTypeFromString(FLAGS_target_type);
      std::unique_ptr<aos::EventLoop> constants_event_loop =
          factory->MakeEventLoop("constants_fetcher", pi_event_loop->node());
      if (FLAGS_base_calibration.empty()) {
        frc971::constants::ConstantsFetcher<y2023::Constants> constants_fetcher(
            constants_event_loop.get());
        *intrinsics =
            aos::RecursiveCopyFlatBuffer(y2023::vision::FindCameraCalibration(
                constants_fetcher.constants(),
                pi_event_loop->node()->name()->string_view()));
      } else {
        *intrinsics = aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
            FLAGS_base_calibration);
      }
      extractor = std::make_unique<Calibration>(
          factory, pi_event_loop.get(), imu_event_loop.get(), FLAGS_pi,
          &intrinsics->message(), target_type, FLAGS_image_channel, data);
    }
  }
};

void Main(int argc, char **argv) {
  CalibrationData data;
  std::optional<uint16_t> pi_number = aos::network::ParsePiNumber(FLAGS_pi);
  CHECK(pi_number);
  const std::string pi_name = absl::StrCat("pi", *pi_number);

  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> calibration =
      aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>::Empty();
  {
    // Now, accumulate all the data into the data object.
    aos::logger::LogReader reader(
        aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
    reader.RemapLoggedChannel<foxglove::CompressedImage>("/pi1/camera");
    reader.RemapLoggedChannel<foxglove::CompressedImage>("/pi2/camera");
    reader.RemapLoggedChannel<foxglove::CompressedImage>("/pi3/camera");
    reader.RemapLoggedChannel<foxglove::CompressedImage>("/pi4/camera");
    reader.RemapLoggedChannel<foxglove::ImageAnnotations>("/pi1/camera");
    reader.RemapLoggedChannel<foxglove::ImageAnnotations>("/pi2/camera");
    reader.RemapLoggedChannel<foxglove::ImageAnnotations>("/pi3/camera");
    reader.RemapLoggedChannel<foxglove::ImageAnnotations>("/pi4/camera");

    aos::SimulatedEventLoopFactory factory(reader.configuration());
    reader.RegisterWithoutStarting(&factory);

    CHECK(aos::configuration::MultiNode(reader.configuration()));

    // Find the nodes we care about.
    const aos::Node *const imu_node =
        aos::configuration::GetNode(factory.configuration(), "imu");

    const aos::Node *const pi_node =
        aos::configuration::GetNode(factory.configuration(), pi_name);

    CalibrationAccumulatorState accumulator_state(&factory, &data,
                                                  &calibration);

    reader.OnStart(imu_node, [&accumulator_state, imu_node, &factory]() {
      accumulator_state.imu_event_loop =
          factory.MakeEventLoop("calibration", imu_node);
      accumulator_state.MaybeMakeExtractor();
    });

    reader.OnStart(imu_node, [&accumulator_state, pi_node, &factory]() {
      accumulator_state.pi_event_loop =
          factory.MakeEventLoop("logger", pi_node);
      accumulator_state.logger_event_loop =
          factory.MakeEventLoop("logger", pi_node);
      accumulator_state.logger = std::make_unique<aos::logger::Logger>(
          accumulator_state.logger_event_loop.get());
      accumulator_state.logger->StartLoggingOnRun(FLAGS_output_logs);
      accumulator_state.MaybeMakeExtractor();
    });

    factory.Run();

    reader.Deregister();
  }

  LOG(INFO) << "Done with event_loop running";
  CHECK(data.imu_samples_size() > 0) << "Didn't get any IMU data";
  CHECK(data.camera_samples_size() > 0) << "Didn't get any camera observations";

  // And now we have it, we can start processing it.
  // NOTE: For y2023, with no turret, pivot == imu

  // Define the mapping that takes imu frame (with z up) to camera frame (with z
  // pointing out)
  const Eigen::Quaterniond R_precam_cam(
      Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ()));
  // Set up initial conditions for the pis that are reasonable
  Eigen::Quaternion<double> nominal_initial_orientation(
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  Eigen::Quaternion<double> nominal_pivot_to_camera(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.75 * M_PI, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d nominal_pivot_to_camera_translation(8.0, 8.0, 0.0);
  Eigen::Quaternion<double> nominal_board_to_world(
      Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()));
  Eigen::Matrix<double, 6, 1> nominal_initial_state =
      Eigen::Matrix<double, 6, 1>::Zero();
  // Set y value to -1 m (approx distance from imu to the board/world)
  nominal_initial_state(1, 0) = 1.0;

  // If available, pull extrinsics from the calibration file
  const calibration::CameraCalibration *calib_msg = &calibration.message();
  if (calib_msg->has_fixed_extrinsics()) {
    LOG(INFO) << "Using extrinsiscs from calibration file as initial condition";
    const cv::Mat extrinsics_cv(
        4, 4, CV_32F,
        const_cast<void *>(static_cast<const void *>(
            calib_msg->fixed_extrinsics()->data()->data())));
    CHECK_EQ(extrinsics_cv.total(),
             calib_msg->fixed_extrinsics()->data()->size());
    Eigen::Matrix4d ext_mat;
    cv::cv2eigen(extrinsics_cv, ext_mat);
    Eigen::Affine3d extrinsics(ext_mat);
    Eigen::Affine3d H_camera_imu = extrinsics.inverse();
    nominal_pivot_to_camera = H_camera_imu.rotation();
    nominal_pivot_to_camera_translation = H_camera_imu.translation();
  }

  CalibrationParameters calibration_parameters;
  calibration_parameters.initial_orientation = nominal_initial_orientation;
  calibration_parameters.pivot_to_camera = nominal_pivot_to_camera;
  calibration_parameters.pivot_to_camera_translation =
      nominal_pivot_to_camera_translation;
  // Board to world rotation
  calibration_parameters.board_to_world = nominal_board_to_world;
  // Initial imu location (and velocity)
  calibration_parameters.initial_state = nominal_initial_state;
  calibration_parameters.has_pivot = false;

  // Show the inverse of pivot_to_camera, since camera_to_pivot tells where
  // the camera is with respect to the pivot frame
  const Eigen::Affine3d nominal_affine_pivot_to_camera =
      Eigen::Translation3d(calibration_parameters.pivot_to_camera_translation) *
      nominal_pivot_to_camera;
  const Eigen::Quaterniond nominal_camera_to_pivot_rotation(
      nominal_affine_pivot_to_camera.inverse().rotation());
  const Eigen::Vector3d nominal_camera_to_pivot_translation(
      nominal_affine_pivot_to_camera.inverse().translation());

  LOG(INFO) << "Initial Conditions for solver.  Assumes:\n"
            << "1) board origin is same as world, but rotated pi/2 about "
               "x-axis, so z points out\n"
            << "2) pivot origin matches imu origin (since there's no turret)\n"
            << "3) camera is offset from pivot (depends on which camera)";

  LOG(INFO) << "Nominal initial_orientation of imu w.r.t. world "
               "(angle-axis vector): "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   nominal_initial_orientation)
                   .transpose();
  LOG(INFO) << "Nominal initial_state: \n"
            << "Position: "
            << nominal_initial_state.block<3, 1>(0, 0).transpose() << "\n"
            << "Velocity: "
            << nominal_initial_state.block<3, 1>(3, 0).transpose();
  LOG(INFO) << "Nominal camera_to_pivot rotation (angle-axis vector): "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   nominal_camera_to_pivot_rotation)
                   .transpose();
  LOG(INFO) << "Nominal camera_to_pivot translation: "
            << nominal_camera_to_pivot_translation.transpose();

  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      solved_extrinsics = Solve(data, &calibration_parameters);
  calibration.mutable_message()->clear_fixed_extrinsics();
  calibration.mutable_message()->clear_turret_extrinsics();
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      merged_calibration = aos::MergeFlatBuffers(&calibration.message(),
                                                 &solved_extrinsics.message());

  if (!FLAGS_output_calibration.empty()) {
    aos::WriteFlatbufferToJson(FLAGS_output_calibration, merged_calibration);
  } else {
    LOG(WARNING) << "Calibration filename not provided, so not saving it";
  }

  LOG(INFO) << "RESULTS OF CALIBRATION SOLVER:";
  std::cout << aos::FlatbufferToJson(&merged_calibration.message())
            << std::endl;

  LOG(INFO) << "initial_orientation of imu w.r.t. world (angle-axis vector): "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.initial_orientation)
                   .transpose();
  LOG(INFO)
      << "initial_state (imu): \n"
      << "Position: "
      << calibration_parameters.initial_state.block<3, 1>(0, 0).transpose()
      << "\n"
      << "Velocity: "
      << calibration_parameters.initial_state.block<3, 1>(3, 0).transpose();

  const Eigen::Affine3d affine_pivot_to_camera =
      Eigen::Translation3d(calibration_parameters.pivot_to_camera_translation) *
      calibration_parameters.pivot_to_camera;
  const Eigen::Affine3d affine_camera_to_pivot =
      affine_pivot_to_camera.inverse();
  const Eigen::Quaterniond camera_to_pivot_rotation(
      affine_camera_to_pivot.rotation());
  const Eigen::Vector3d camera_to_pivot_translation(
      affine_camera_to_pivot.translation());
  LOG(INFO) << "camera to pivot(imu) rotation (angle-axis vec): "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   camera_to_pivot_rotation)
                   .transpose();
  LOG(INFO) << "camera to pivot(imu) translation: "
            << camera_to_pivot_translation.transpose();
  LOG(INFO) << "board_to_world (rotation) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.board_to_world)
                   .transpose();
  LOG(INFO) << "accelerometer bias "
            << calibration_parameters.accelerometer_bias.transpose();
  LOG(INFO) << "gyro_bias " << calibration_parameters.gyro_bias.transpose();
  LOG(INFO) << "gravity " << 9.81 * calibration_parameters.gravity_scalar;

  LOG(INFO) << "Checking delta from nominal (initial condition) to solved "
               "values:";
  LOG(INFO) << "nominal_pivot_to_camera rotation: "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   R_precam_cam * nominal_pivot_to_camera)
                   .transpose();
  LOG(INFO) << "solved pivot_to_camera rotation: "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   R_precam_cam * calibration_parameters.pivot_to_camera)
                   .transpose();

  LOG(INFO) << "pivot_to_camera rotation delta (zero if the IC's match the "
               "solved value) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.pivot_to_camera *
                   nominal_pivot_to_camera.inverse())
                   .transpose();
  LOG(INFO) << "board_to_world rotation change "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.board_to_world *
                   nominal_board_to_world.inverse())
                   .transpose();

  if (FLAGS_visualize) {
    LOG(INFO) << "Showing visualization";
    Visualize(data, calibration_parameters);
  }

  if (FLAGS_plot) {
    Plot(data, calibration_parameters);
  }
}  // namespace vision

}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  frc971::vision::Main(argc, argv);
}
