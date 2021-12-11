#ifndef Y2020_VISION_CALIBRATION_ACCUMULATOR_H_
#define Y2020_VISION_CALIBRATION_ACCUMULATOR_H_

#include <vector>

#include "Eigen/Dense"
#include "aos/events/simulated_event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2020/vision/charuco_lib.h"

namespace frc971 {
namespace vision {

class CalibrationDataObserver {
 public:
  virtual void UpdateCamera(aos::distributed_clock::time_point t,
                            std::pair<Eigen::Vector3d, Eigen::Vector3d> rt) = 0;

  virtual void UpdateIMU(aos::distributed_clock::time_point t,
                         std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) = 0;
};

// Class to both accumulate and replay camera and IMU data in time order.
class CalibrationData {
 public:
  // Adds a camera/charuco detection to the list at the provided time.
  // This has only been tested with a charuco board.
  void AddCameraPose(aos::distributed_clock::time_point distributed_now,
                     Eigen::Vector3d rvec, Eigen::Vector3d tvec);

  // Adds an IMU point to the list at the provided time.
  void AddImu(aos::distributed_clock::time_point distributed_now,
              Eigen::Vector3d gyro, Eigen::Vector3d accel);

  // Processes the data points by calling UpdateCamera and UpdateIMU in time
  // order.
  void ReviewData(CalibrationDataObserver *observer);

  size_t camera_samples_size() const { return rot_trans_points_.size(); }

 private:
  std::vector<std::pair<aos::distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      imu_points_;

  // Store pose samples as timestamp, along with
  // pair of rotation, translation vectors
  std::vector<std::pair<aos::distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      rot_trans_points_;
};

// Class to register image and IMU callbacks in AOS and route them to the
// corresponding CalibrationData class.
class Calibration {
 public:
  Calibration(aos::SimulatedEventLoopFactory *event_loop_factory,
              aos::EventLoop *image_event_loop, aos::EventLoop *imu_event_loop,
              std::string_view pi, CalibrationData *data);

  // Processes a charuco detection.
  void HandleCharuco(cv::Mat rgb_image,
                     const aos::monotonic_clock::time_point eof,
                     std::vector<int> /*charuco_ids*/,
                     std::vector<cv::Point2f> /*charuco_corners*/, bool valid,
                     Eigen::Vector3d rvec_eigen, Eigen::Vector3d tvec_eigen);

  // Processes an IMU reading.
  void HandleIMU(const frc971::IMUValues *imu);

 private:
  aos::EventLoop *image_event_loop_;
  aos::NodeEventLoopFactory *image_factory_;
  aos::EventLoop *imu_event_loop_;
  aos::NodeEventLoopFactory *imu_factory_;

  CharucoExtractor charuco_extractor_;

  CalibrationData *data_;

  frc971::IMUValuesT last_value_;
};

}  // namespace vision
}  // namespace frc971

#endif  // Y2020_VISION_CALIBRATION_ACCUMULATOR_H_
