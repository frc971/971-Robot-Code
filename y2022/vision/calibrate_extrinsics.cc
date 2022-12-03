#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/strings/str_format.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/extrinsics_calibration.h"
#include "frc971/vision/vision_generated.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

DEFINE_string(pi, "pi-7971-2", "Pi name to calibrate.");
DEFINE_bool(plot, false, "Whether to plot the resulting data.");
DEFINE_bool(visualize, false, "Whether to visualize the resulting data.");
DEFINE_bool(turret, true, "If true, the camera is on the turret");

namespace frc971 {
namespace vision {
namespace chrono = std::chrono;
using aos::distributed_clock;
using aos::monotonic_clock;

// TODO(austin): Source of IMU data?  Is it the same?
// TODO(austin): Intrinsics data?

void Main(int argc, char **argv) {
  CalibrationData data;

  {
    // Now, accumulate all the data into the data object.
    aos::logger::LogReader reader(
        aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));

    aos::SimulatedEventLoopFactory factory(reader.configuration());
    reader.Register(&factory);

    CHECK(aos::configuration::MultiNode(reader.configuration()));

    // Find the nodes we care about.
    const aos::Node *const imu_node =
        aos::configuration::GetNode(factory.configuration(), "imu");
    const aos::Node *const roborio_node =
        aos::configuration::GetNode(factory.configuration(), "roborio");

    std::optional<uint16_t> pi_number = aos::network::ParsePiNumber(FLAGS_pi);
    CHECK(pi_number);
    LOG(INFO) << "Pi " << *pi_number;
    const aos::Node *const pi_node = aos::configuration::GetNode(
        factory.configuration(), absl::StrCat("pi", *pi_number));

    LOG(INFO) << "imu " << aos::FlatbufferToJson(imu_node);
    LOG(INFO) << "roboRIO " << aos::FlatbufferToJson(roborio_node);
    LOG(INFO) << "Pi " << aos::FlatbufferToJson(pi_node);

    std::unique_ptr<aos::EventLoop> imu_event_loop =
        factory.MakeEventLoop("calibration", imu_node);
    std::unique_ptr<aos::EventLoop> roborio_event_loop =
        factory.MakeEventLoop("calibration", roborio_node);
    std::unique_ptr<aos::EventLoop> pi_event_loop =
        factory.MakeEventLoop("calibration", pi_node);

    // Now, hook Calibration up to everything.
    Calibration extractor(&factory, pi_event_loop.get(), imu_event_loop.get(),
                          FLAGS_pi, &data);

    if (FLAGS_turret) {
      aos::NodeEventLoopFactory *roborio_factory =
          factory.GetNodeEventLoopFactory(roborio_node->name()->string_view());
      roborio_event_loop->MakeWatcher(
          "/superstructure",
          [roborio_factory, roborio_event_loop = roborio_event_loop.get(),
           &data](const y2022::control_loops::superstructure::Status &status) {
            data.AddTurret(
                roborio_factory->ToDistributedClock(
                    roborio_event_loop->context().monotonic_event_time),
                Eigen::Vector2d(status.turret()->position(),
                                status.turret()->velocity()));
          });
    }

    factory.Run();

    reader.Deregister();
  }

  LOG(INFO) << "Done with event_loop running";
  CHECK(data.imu_samples_size() > 0) << "Didn't get any IMU data";
  CHECK(data.camera_samples_size() > 0) << "Didn't get any camera observations";

  // And now we have it, we can start processing it.
  const Eigen::Quaternion<double> nominal_initial_orientation(
      frc971::controls::ToQuaternionFromRotationVector(
          Eigen::Vector3d(0.0, 0.0, M_PI)));
  const Eigen::Quaternion<double> nominal_pivot_to_camera(
      Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()));
  const Eigen::Quaternion<double> nominal_pivot_to_imu(
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));
  const Eigen::Quaternion<double> nominal_board_to_world(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()));
  Eigen::Matrix<double, 6, 1> nominal_initial_state =
      Eigen::Matrix<double, 6, 1>::Zero();
  // Set x value to 0.5 m (center view on the board)
  // nominal_initial_state(0, 0) = 0.5;
  // Set y value to -1 m (approx distance from imu to board/world)
  nominal_initial_state(1, 0) = -1.0;

  CalibrationParameters calibration_parameters;
  calibration_parameters.initial_orientation = nominal_initial_orientation;
  calibration_parameters.pivot_to_camera = nominal_pivot_to_camera;
  calibration_parameters.pivot_to_imu = nominal_pivot_to_imu;
  calibration_parameters.board_to_world = nominal_board_to_world;
  calibration_parameters.initial_state = nominal_initial_state;

  // Show the inverse of pivot_to_camera, since camera_to_pivot tells where the
  // camera is with respect to the pivot frame
  const Eigen::Affine3d nominal_affine_pivot_to_camera =
      Eigen::Translation3d(calibration_parameters.pivot_to_camera_translation) *
      nominal_pivot_to_camera;
  const Eigen::Quaterniond nominal_camera_to_pivot_rotation(
      nominal_affine_pivot_to_camera.inverse().rotation());
  const Eigen::Vector3d nominal_camera_to_pivot_translation(
      nominal_affine_pivot_to_camera.inverse().translation());

  if (data.turret_samples_size() > 0) {
    LOG(INFO) << "Have turret, so using pivot setup";
    calibration_parameters.has_pivot = true;
  }

  LOG(INFO) << "Initial Conditions for solver.  Assumes:\n"
            << "1) board origin is same as world, but rotated pi/2 about "
               "x-axis, so z points out\n"
            << "2) pivot origin matches imu origin\n"
            << "3) camera is offset from pivot (depends on which camera)";

  LOG(INFO)
      << "Nominal initial_orientation of imu w.r.t. world (angle-axis vector): "
      << frc971::controls::ToRotationVectorFromQuaternion(
             nominal_initial_orientation)
             .transpose();
  LOG(INFO) << "Nominal initial_state: \n"
            << "Position: "
            << nominal_initial_state.block<3, 1>(0, 0).transpose() << "\n"
            << "Velocity: "
            << nominal_initial_state.block<3, 1>(3, 0).transpose();
  LOG(INFO) << "Nominal pivot_to_imu (angle-axis vector) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.pivot_to_imu)
                   .transpose();
  LOG(INFO) << "Nominal pivot_to_imu translation: "
            << calibration_parameters.pivot_to_imu_translation.transpose();
  // TODO<Jim>: Might be nice to take out the rotation component that maps into
  // camera image coordinates (with x right, y down, z forward)
  LOG(INFO) << "Nominal camera_to_pivot (angle-axis vector): "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   nominal_camera_to_pivot_rotation)
                   .transpose();
  LOG(INFO) << "Nominal camera_to_pivot translation: "
            << nominal_camera_to_pivot_translation.transpose();

  Solve(data, &calibration_parameters);

  LOG(INFO) << "RESULTS OF CALIBRATION SOLVER:";
  LOG(INFO) << "initial_orientation of imu w.r.t. world (angle-axis vector): "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.initial_orientation)
                   .transpose();
  LOG(INFO)
      << "initial_state: \n"
      << "Position: "
      << calibration_parameters.initial_state.block<3, 1>(0, 0).transpose()
      << "\n"
      << "Velocity: "
      << calibration_parameters.initial_state.block<3, 1>(3, 0).transpose();

  LOG(INFO) << "pivot_to_imu rotation (angle-axis vec) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.pivot_to_imu)
                   .transpose();
  LOG(INFO) << "pivot_to_imu_translation "
            << calibration_parameters.pivot_to_imu_translation.transpose();
  const Eigen::Affine3d affine_pivot_to_camera =
      Eigen::Translation3d(calibration_parameters.pivot_to_camera_translation) *
      calibration_parameters.pivot_to_camera;
  const Eigen::Quaterniond camera_to_pivot_rotation(
      affine_pivot_to_camera.inverse().rotation());
  const Eigen::Vector3d camera_to_pivot_translation(
      affine_pivot_to_camera.inverse().translation());
  LOG(INFO) << "camera to pivot (angle-axis vec): "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   camera_to_pivot_rotation)
                   .transpose();
  LOG(INFO) << "camera to pivot translation: "
            << camera_to_pivot_translation.transpose();
  LOG(INFO) << "board_to_world (rotation) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.board_to_world)
                   .transpose();
  LOG(INFO) << "accelerometer bias "
            << calibration_parameters.accelerometer_bias.transpose();
  LOG(INFO) << "gyro_bias " << calibration_parameters.gyro_bias.transpose();
  LOG(INFO) << "gravity " << 9.81 * calibration_parameters.gravity_scalar;

  LOG(INFO) << "pivot_to_camera change "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.pivot_to_camera *
                   nominal_pivot_to_camera.inverse())
                   .transpose();
  LOG(INFO) << "board_to_world delta "
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
}

}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  frc971::vision::Main(argc, argv);
}
