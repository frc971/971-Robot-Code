#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "frc971/wpilib/imu_batch_generated.h"
#include "absl/strings/str_format.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "y2020/vision/charuco_lib.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"
#include "y2020/vision/vision_generated.h"
#include "y2020/vision/charuco_lib.h"

DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_string(pi, "pi-7971-2", "Pi name to calibrate.");
DEFINE_bool(display_undistorted, false,
            "If true, display the undistorted image.");

namespace frc971 {
namespace vision {
namespace chrono = std::chrono;
using aos::distributed_clock;
using aos::monotonic_clock;

// Class to both accumulate and replay camera and IMU data in time order.
class CalibrationData {
 public:
  CalibrationData()
      : x_hat_(Eigen::Matrix<double, 9, 1>::Zero()),
        q_(Eigen::Matrix<double, 9, 9>::Zero()) {}

  // Adds an IMU point to the list at the provided time.
  void AddImu(distributed_clock::time_point distributed_now,
              Eigen::Vector3d gyro, Eigen::Vector3d accel) {
    imu_points_.emplace_back(distributed_now, std::make_pair(gyro, accel));
  }

  // Adds a camera/charuco detection to the list at the provided time.
  void AddCharuco(distributed_clock::time_point distributed_now,
                  Eigen::Vector3d rvec, Eigen::Vector3d tvec) {
    rot_trans_points_.emplace_back(distributed_now, std::make_pair(rvec, tvec));
  }

  // Processes the data points by calling UpdateCamera and UpdateIMU in time
  // order.
  void ReviewData() {
    size_t next_imu_point = 0;
    size_t next_camera_point = 0;
    while (true) {
      if (next_imu_point != imu_points_.size()) {
        // There aren't that many combinations, so just brute force them all
        // rather than being too clever.
        if (next_camera_point != rot_trans_points_.size()) {
          if (imu_points_[next_imu_point].first >
              rot_trans_points_[next_camera_point].first) {
            // Camera!
            UpdateCamera(rot_trans_points_[next_camera_point].first,
                         rot_trans_points_[next_camera_point].second);
            ++next_camera_point;
          } else {
            // IMU!
            UpdateIMU(imu_points_[next_imu_point].first,
                      imu_points_[next_imu_point].second);
            ++next_imu_point;
          }
        } else {
          if (next_camera_point != rot_trans_points_.size()) {
            // Camera!
            UpdateCamera(rot_trans_points_[next_camera_point].first,
                         rot_trans_points_[next_camera_point].second);
            ++next_camera_point;
          } else {
            // Nothing left for either list of points, so we are done.
            break;
          }
        }
      }
    }
  }

  void UpdateCamera(distributed_clock::time_point t,
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> rt) {
    LOG(INFO) << t << " Camera " << rt.second.transpose();
  }

  void UpdateIMU(distributed_clock::time_point t,
                 std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) {
    LOG(INFO) << t << " IMU " << wa.first.transpose();
  }

 private:
  // TODO(austin): Actually use these.  Or make a new "callback" object which has these.
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

  std::vector<std::pair<distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      imu_points_;

  // Store pose samples as timestamp, along with
  // pair of rotation, translation vectors
  std::vector<std::pair<distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      rot_trans_points_;
};

// Class to register image and IMU callbacks in AOS and route them to the
// corresponding CalibrationData class.
class Calibration {
 public:
  Calibration(aos::SimulatedEventLoopFactory *event_loop_factory,
              aos::EventLoop *image_event_loop, aos::EventLoop *imu_event_loop,
              std::string_view pi, CalibrationData *data)
      : image_event_loop_(image_event_loop),
        image_factory_(event_loop_factory->GetNodeEventLoopFactory(
            image_event_loop_->node())),
        imu_event_loop_(imu_event_loop),
        imu_factory_(event_loop_factory->GetNodeEventLoopFactory(
            imu_event_loop_->node())),
        charuco_extractor_(
            image_event_loop_, pi,
            [this](cv::Mat rgb_image, monotonic_clock::time_point eof,
                   std::vector<int> charuco_ids,
                   std::vector<cv::Point2f> charuco_corners, bool valid,
                   Eigen::Vector3d rvec_eigen, Eigen::Vector3d tvec_eigen) {
              HandleCharuco(rgb_image, eof, charuco_ids, charuco_corners, valid,
                            rvec_eigen, tvec_eigen);
            }),
        data_(data) {
    imu_event_loop_->MakeWatcher(
        "/drivetrain", [this](const frc971::IMUValuesBatch &imu) {
          if (!imu.has_readings()) {
            return;
          }
          for (const frc971::IMUValues *value : *imu.readings()) {
            HandleIMU(value);
          }
        });
  }

  // Processes a charuco detection.
  void HandleCharuco(cv::Mat rgb_image, const monotonic_clock::time_point eof,
                     std::vector<int> /*charuco_ids*/,
                     std::vector<cv::Point2f> /*charuco_corners*/, bool valid,
                     Eigen::Vector3d rvec_eigen, Eigen::Vector3d tvec_eigen) {
    if (valid) {
      Eigen::Quaternion<double> rotation(
          frc971::controls::ToQuaternionFromRotationVector(rvec_eigen));
      Eigen::Translation3d translation(tvec_eigen);

      const Eigen::Affine3d board_to_camera = translation * rotation;
      (void)board_to_camera;

      // TODO(austin): Need a gravity vector input.
      //
      // TODO(austin): Need a state, covariance, and model.
      //
      // TODO(austin): Need to record all the values out of a log and run it
      // as a batch run so we can feed it into ceres.

      // LOG(INFO) << "rotation " << rotation.matrix();
      // LOG(INFO) << "translation " << translation.matrix();
      // Z -> up
      // Y -> away from cameras 2 and 3
      // X -> left
      Eigen::Vector3d imu(last_value_.accelerometer_x,
                          last_value_.accelerometer_y,
                          last_value_.accelerometer_z);

      // For cameras 2 and 3...
      Eigen::Quaternion<double> imu_to_camera(
          Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()));

      Eigen::Quaternion<double> board_to_world(
          Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()));

      Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]",
                               "[", "]");

      LOG(INFO);
      LOG(INFO) << "World Gravity "
                << (board_to_world * rotation.inverse() * imu_to_camera * imu)
                       .transpose()
                       .format(HeavyFmt);
      LOG(INFO) << "Board Gravity "
                << (rotation.inverse() * imu_to_camera * imu)
                       .transpose()
                       .format(HeavyFmt);
      LOG(INFO) << "Camera Gravity "
                << (imu_to_camera * imu).transpose().format(HeavyFmt);
      LOG(INFO) << "IMU Gravity " << imu.transpose().format(HeavyFmt);

      const double age_double =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              image_event_loop_->monotonic_now() - eof)
              .count();
      LOG(INFO) << std::fixed << std::setprecision(6) << "Age: " << age_double
                << ", Pose is R:" << rvec_eigen.transpose().format(HeavyFmt)
                << " T:" << tvec_eigen.transpose().format(HeavyFmt);

      data_->AddCharuco(image_factory_->ToDistributedClock(eof), rvec_eigen,
                        tvec_eigen);
    }

    cv::imshow("Display", rgb_image);

    if (FLAGS_display_undistorted) {
      const cv::Size image_size(rgb_image.cols, rgb_image.rows);
      cv::Mat undistorted_rgb_image(image_size, CV_8UC3);
      cv::undistort(rgb_image, undistorted_rgb_image,
                    charuco_extractor_.camera_matrix(),
                    charuco_extractor_.dist_coeffs());

      cv::imshow("Display undist", undistorted_rgb_image);
    }

    int keystroke = cv::waitKey(1);
    if ((keystroke & 0xFF) == static_cast<int>('q')) {
      // image_event_loop_->Exit();
    }
  }

  // Processes an IMU reading.
  void HandleIMU(const frc971::IMUValues *imu) {
    VLOG(1) << "IMU " << imu;
    imu->UnPackTo(&last_value_);
    Eigen::Vector3d gyro(last_value_.gyro_x, last_value_.gyro_y,
                         last_value_.gyro_z);
    Eigen::Vector3d accel(last_value_.accelerometer_x,
                          last_value_.accelerometer_y,
                          last_value_.accelerometer_z);

    data_->AddImu(imu_factory_->ToDistributedClock(monotonic_clock::time_point(
                      chrono::nanoseconds(imu->monotonic_timestamp_ns()))),
                  gyro, accel);
  }

  frc971::IMUValuesT last_value_;

 private:
  aos::EventLoop *image_event_loop_;
  aos::NodeEventLoopFactory *image_factory_;
  aos::EventLoop *imu_event_loop_;
  aos::NodeEventLoopFactory *imu_factory_;

  CharucoExtractor charuco_extractor_;

  CalibrationData *data_;
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
  data.ReviewData();
}

}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  frc971::vision::Main(argc, argv);
}
