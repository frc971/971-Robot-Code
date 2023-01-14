#ifndef FRC971_VISION_CALIBRATION_ACCUMULATOR_H_
#define FRC971_VISION_CALIBRATION_ACCUMULATOR_H_

#include <vector>

#include "Eigen/Dense"
#include "aos/events/simulated_event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/wpilib/imu_batch_generated.h"

namespace frc971 {
namespace vision {

// This class provides an interface for an application to be notified of all
// camera and IMU samples in order with the correct timestamps.
class CalibrationDataObserver {
 public:
  // Observes a camera sample at the corresponding time t, and with the
  // corresponding rotation and translation vectors rt.
  virtual void UpdateCamera(aos::distributed_clock::time_point t,
                            std::pair<Eigen::Vector3d, Eigen::Vector3d> rt) = 0;

  // Observes an IMU sample at the corresponding time t, and with the
  // corresponding angular velocity and linear acceleration vectors wa.
  virtual void UpdateIMU(aos::distributed_clock::time_point t,
                         std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) = 0;

  // Observes a turret sample at the corresponding time t, and with the
  // corresponding state.
  virtual void UpdateTurret(aos::distributed_clock::time_point t,
                            Eigen::Vector2d state) = 0;
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

  // Adds a turret reading (position; velocity) to the list at the provided
  // time.
  void AddTurret(aos::distributed_clock::time_point distributed_now,
                 Eigen::Vector2d state);

  // Processes the data points by calling UpdateCamera and UpdateIMU in time
  // order.
  void ReviewData(CalibrationDataObserver *observer) const;

  size_t camera_samples_size() const { return rot_trans_points_.size(); }

  size_t imu_samples_size() const { return imu_points_.size(); }

  size_t turret_samples_size() const { return turret_points_.size(); }

 private:
  std::vector<std::pair<aos::distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      imu_points_;

  // Store pose samples as timestamp, along with
  // pair of rotation, translation vectors
  std::vector<std::pair<aos::distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      rot_trans_points_;

  // Turret state as a timestamp and [x, v].
  std::vector<std::pair<aos::distributed_clock::time_point, Eigen::Vector2d>>
      turret_points_;
};

// Class to register image and IMU callbacks in AOS and route them to the
// corresponding CalibrationData class.
class Calibration {
 public:
  Calibration(aos::SimulatedEventLoopFactory *event_loop_factory,
              aos::EventLoop *image_event_loop, aos::EventLoop *imu_event_loop,
              std::string_view pi, TargetType target_type,
              std::string_view image_channel, CalibrationData *data);

  // Processes a charuco detection that is returned from charuco_lib.
  // For a valid detection(s), it stores camera observation
  // Also optionally displays and saves annotated images based on visualize and
  // save_path flags, respectively
  void HandleCharuco(cv::Mat rgb_image,
                     const aos::monotonic_clock::time_point eof,
                     std::vector<cv::Vec4i> /*charuco_ids*/,
                     std::vector<std::vector<cv::Point2f>> /*charuco_corners*/,
                     bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                     std::vector<Eigen::Vector3d> tvecs_eigen);

  // Processes an IMU reading by storing for later processing
  void HandleIMU(const frc971::IMUValues *imu);

 private:
  aos::EventLoop *image_event_loop_;
  aos::NodeEventLoopFactory *image_factory_;
  aos::EventLoop *imu_event_loop_;
  aos::NodeEventLoopFactory *imu_factory_;

  CharucoExtractor charuco_extractor_;
  ImageCallback image_callback_;

  CalibrationData *data_;

  frc971::IMUValuesT last_value_;
};

}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_CALIBRATION_ACCUMULATOR_H_
