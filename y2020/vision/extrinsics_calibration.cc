#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "absl/strings/str_format.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2020/vision/calibration_accumulator.h"
#include "y2020/vision/charuco_lib.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"
#include "y2020/vision/vision_generated.h"

DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_string(pi, "pi-7971-2", "Pi name to calibrate.");

namespace frc971 {
namespace vision {
namespace chrono = std::chrono;
using aos::distributed_clock;
using aos::monotonic_clock;

class PoseFilter : public CalibrationDataObserver {
 public:
  PoseFilter()
      : accel_(Eigen::Matrix<double, 3, 1>::Zero()),
        omega_(Eigen::Matrix<double, 3, 1>::Zero()),
        x_hat_(Eigen::Matrix<double, 9, 1>::Zero()),
        q_(Eigen::Matrix<double, 9, 9>::Zero()) {}

  void UpdateCamera(distributed_clock::time_point t,
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> rt) override {
    Integrate(t);
    // TODO(austin): take the sample.
    LOG(INFO) << t << " Camera " << rt.second.transpose();
  }

  void UpdateIMU(distributed_clock::time_point t,
                 std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) override {
    Integrate(t);
    omega_ = wa.first;
    accel_ = wa.second;
    LOG(INFO) << t << " IMU " << wa.first.transpose();
  }

 private:
  void Integrate(distributed_clock::time_point t) { LOG(INFO) << t; }

  Eigen::Matrix<double, 3, 1> accel_;
  Eigen::Matrix<double, 3, 1> omega_;

  // TODO(austin): Actually use these.  Or make a new "callback" object which
  // has these.
  Eigen::Matrix<double, 9, 1> x_hat_;
  Eigen::Matrix<double, 9, 9> q_;

  // Proposed filter states:
  // States:
  //   xyz position
  //   xyz velocity
  //   orientation rotation vector
  //
  // Inputs
  //   xyz accel
  //   angular rates
  //
  // Measurement:
  //   xyz position
  //   orientation rotation vector
};

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

  {
    PoseFilter filter;
    data.ReviewData(&filter);
  }
}

}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  frc971::vision::Main(argc, argv);
}
