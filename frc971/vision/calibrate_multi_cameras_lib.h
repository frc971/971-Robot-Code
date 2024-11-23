#ifndef FRC971_VISION_CALIBRATE_MULTI_CAMERAS_LIB_H_
#define FRC971_VISION_CALIBRATE_MULTI_CAMERAS_LIB_H_

#include <numeric>
#include <string>

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/util/mcap_logger.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/extrinsics_calibration.h"
#include "frc971/vision/target_mapper.h"
#include "frc971/vision/vision_util_lib.h"
#include "frc971/vision/visualize_robot.h"
// clang-format off
// OpenCV eigen files must be included after Eigen includes
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
// clang-format on
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/vision/vision_util_lib.h"

ABSL_DECLARE_FLAG(float, max_pose_error);
ABSL_DECLARE_FLAG(double, max_pose_error_ratio);
ABSL_DECLARE_FLAG(int32_t, team_number);
ABSL_DECLARE_FLAG(uint64_t, wait_key);

namespace frc971::vision {

static constexpr double kDefaultImagePeriodMs =
    1.0 / 60.0 * 1000.0;  // Image capture period in ms

// Change reference frame from camera to robot
Eigen::Affine3d CameraToRobotDetection(Eigen::Affine3d H_camera_target,
                                       Eigen::Affine3d extrinsics);

struct TimestampedCameraDetection {
  aos::distributed_clock::time_point time;
  // Pose of target relative to robot
  Eigen::Affine3d H_camera_target;
  // name of pi
  std::string camera_name;
  int board_id;
};

// Structure to store node name (e.g., orin1, imu), number, and a usable string
struct CameraNode {
  std::string node_name;
  int camera_number;

  inline const std::string camera_name() const {
    return "/" + node_name + "/camera" + std::to_string(camera_number);
  }
};

std::vector<CameraNode> CreateNodeList();

std::map<std::string, int> CreateOrderingMap(
    std::vector<CameraNode> &node_list);

// Helper function to compute average pose when supplied with list
// of TimestampedCameraDetection's
Eigen::Affine3d ComputeAveragePose(
    std::vector<TimestampedCameraDetection> &pose_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr);

// Do outlier rejection.  Given a list of poses, compute the
// mean and standard deviation, and throw out those more than
// FLAGS_outlier_std_devs standard deviations away from the mean.
// Repeat for the desired number of iterations or until we don't throw
// out any more outliers
void RemoveOutliers(std::vector<TimestampedCameraDetection> &pose_list,
                    int num_iterations);

// Take in list of poses from a camera observation and add to running list
// One of two options:
// 1) We see two boards in one view-- store this to get an estimate of
// the offset between the two boards
// 2) We see just one board-- save this and try to pair it with a previous
// observation from another camera
void HandlePoses(
    cv::Mat rgb_image, std::vector<TargetMapper::TargetPose> target_poses,
    aos::distributed_clock::time_point distributed_eof, std::string camera_name,
    TimestampedCameraDetection &last_observation,
    std::vector<std::pair<TimestampedCameraDetection,
                          TimestampedCameraDetection>> &detection_list,
    std::vector<TimestampedCameraDetection> &two_board_extrinsics_list,
    VisualizeRobot &vis_robot_,
    std::map<std::string, cv::Scalar> const &camera_colors,
    std::map<std::string, int> &ordering_map, double image_period_ms,
    int display_count);

void HandleTargetMap(
    const TargetMap &map, aos::distributed_clock::time_point distributed_eof,
    std::string camera_name,
    std::map<std::string, aos::distributed_clock::time_point> &last_eofs_debug,
    TimestampedCameraDetection &last_observation,
    std::vector<std::pair<TimestampedCameraDetection,
                          TimestampedCameraDetection>> &detection_list,
    std::vector<TimestampedCameraDetection> &two_board_extrinsics_list,
    VisualizeRobot &vis_robot_,
    std::map<std::string, cv::Scalar> &camera_colors,
    std::map<std::string, int> &ordering_map, double image_period_ms,
    int display_count);

void HandleImage(
    aos::EventLoop *event_loop, cv::Mat rgb_image,
    const aos::monotonic_clock::time_point eof,
    aos::distributed_clock::time_point distributed_eof,
    frc971::vision::CharucoExtractor &charuco_extractor,
    std::string camera_name, TimestampedCameraDetection &last_observation,
    std::vector<std::pair<TimestampedCameraDetection,
                          TimestampedCameraDetection>> &detection_list,
    std::vector<TimestampedCameraDetection> &two_board_extrinsics_list,
    VisualizeRobot &vis_robot_,
    std::map<std::string, cv::Scalar> &camera_colors,
    std::map<std::string, int> &ordering_map, double image_period_ms,
    int display_count);

void WriteExtrinsicFile(Eigen::Affine3d extrinsic, CameraNode camera_node,
                        const calibration::CameraCalibration *original_cal);

void ExtrinsicsMain(std::vector<CameraNode> &node_list,
                    std::function<const calibration::CameraCalibration *(
                        aos::EventLoop *const, std::string, int)>
                        find_calibration,
                    std::map<std::string, cv::Scalar> const &camera_colors,
                    aos::logger::LogReader *reader,
                    std::map<std::string, int> &ordering_map,
                    int remove_outliers_iterations = 1,
                    double image_period_ms = kDefaultImagePeriodMs);
}  // namespace frc971::vision

#endif  // FRC971_VISION_CALIBRATE_MULTI_CAMERAS_H_
