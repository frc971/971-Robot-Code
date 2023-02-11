#include "frc971/vision/extrinsics_calibration.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/strings/str_format.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/vision_generated.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

DEFINE_string(pi, "pi-7971-2", "Pi name to calibrate.");
DEFINE_bool(plot, false, "Whether to plot the resulting data.");
DEFINE_bool(turret, false, "If true, the camera is on the turret");
DEFINE_string(base_intrinsics, "",
              "Intrinsics to use for extrinsics calibration.");

namespace frc971 {
namespace vision {
namespace chrono = std::chrono;
using aos::distributed_clock;
using aos::monotonic_clock;

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

    aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> intrinsics =
        aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
            FLAGS_base_intrinsics);
    // Now, hook Calibration up to everything.
    Calibration extractor(&factory, pi_event_loop.get(), imu_event_loop.get(),
                          FLAGS_pi, &intrinsics.message(), TargetType::kCharuco,
                          "/camera", &data);

    if (FLAGS_turret) {
      aos::NodeEventLoopFactory *roborio_factory =
          factory.GetNodeEventLoopFactory(roborio_node->name()->string_view());
      roborio_event_loop->MakeWatcher(
          "/superstructure",
          [roborio_factory, roborio_event_loop = roborio_event_loop.get(),
           &data](const y2020::control_loops::superstructure::Status &status) {
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
  // Set y value to -1 m (approx distance from imu to board/world
  nominal_initial_state(1, 0) = -1.0;

  CalibrationParameters calibration_parameters;
  calibration_parameters.initial_orientation = nominal_initial_orientation;
  calibration_parameters.pivot_to_camera = nominal_pivot_to_camera;
  calibration_parameters.pivot_to_imu = nominal_pivot_to_imu;
  calibration_parameters.board_to_world = nominal_board_to_world;
  calibration_parameters.initial_state = nominal_initial_state;
  if (data.turret_samples_size() > 0) {
    LOG(INFO) << "Have turret, so using pivot setup";
    calibration_parameters.has_pivot = true;
  }

  Solve(data, &calibration_parameters);
  LOG(INFO) << "Nominal initial_orientation "
            << nominal_initial_orientation.coeffs().transpose();
  LOG(INFO) << "Nominal pivot_to_camera "
            << nominal_pivot_to_camera.coeffs().transpose();
  LOG(INFO) << "Nominal pivot_to_camera (rot-xyz) "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   nominal_pivot_to_camera)
                   .transpose();
  LOG(INFO) << "pivot_to_camera change "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.pivot_to_camera *
                   nominal_pivot_to_camera.inverse())
                   .transpose();
  LOG(INFO) << "Nominal pivot_to_imu "
            << nominal_pivot_to_imu.coeffs().transpose();
  LOG(INFO) << "board_to_world delta "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.board_to_world *
                   nominal_board_to_world.inverse())
                   .transpose();

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
