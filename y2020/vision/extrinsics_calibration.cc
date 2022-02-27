#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "absl/strings/str_format.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/vision_generated.h"
#include "frc971/vision/extrinsics_calibration.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_string(pi, "pi-7971-2", "Pi name to calibrate.");
DEFINE_bool(plot, false, "Whether to plot the resulting data.");

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
    const aos::Node *const roborio_node =
        aos::configuration::GetNode(factory.configuration(), "roborio");

    std::optional<uint16_t> pi_number = aos::network::ParsePiNumber(FLAGS_pi);
    CHECK(pi_number);
    LOG(INFO) << "Pi " << *pi_number;
    const aos::Node *const pi_node = aos::configuration::GetNode(
        factory.configuration(), absl::StrCat("pi", *pi_number));

    LOG(INFO) << "roboRIO " << aos::FlatbufferToJson(roborio_node);
    LOG(INFO) << "Pi " << aos::FlatbufferToJson(pi_node);

    std::unique_ptr<aos::EventLoop> roborio_event_loop =
        factory.MakeEventLoop("calibration", roborio_node);
    std::unique_ptr<aos::EventLoop> pi_event_loop =
        factory.MakeEventLoop("calibration", pi_node);

    // Now, hook Calibration up to everything.
    Calibration extractor(&factory, pi_event_loop.get(),
                          roborio_event_loop.get(), FLAGS_pi, &data);

    factory.Run();

    reader.Deregister();
  }

  LOG(INFO) << "Done with event_loop running";
  // And now we have it, we can start processing it.

  const Eigen::Quaternion<double> nominal_initial_orientation(
      frc971::controls::ToQuaternionFromRotationVector(
          Eigen::Vector3d(0.0, 0.0, M_PI)));
  const Eigen::Quaternion<double> nominal_imu_to_camera(
      Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()));
  const Eigen::Quaternion<double> nominal_board_to_world(
      Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()));

  CalibrationParameters calibration_parameters;
  calibration_parameters.initial_orientation = nominal_initial_orientation;
  calibration_parameters.imu_to_camera = nominal_imu_to_camera;
  calibration_parameters.board_to_world = nominal_board_to_world;

  Solve(data, &calibration_parameters);
  LOG(INFO) << "Nominal initial_orientation "
            << nominal_initial_orientation.coeffs().transpose();
  LOG(INFO) << "Nominal imu_to_camera "
            << nominal_imu_to_camera.coeffs().transpose();

  LOG(INFO) << "imu_to_camera delta "
            << frc971::controls::ToRotationVectorFromQuaternion(
                   calibration_parameters.imu_to_camera *
                   nominal_imu_to_camera.inverse())
                   .transpose();
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
