#ifndef FRC971_VISION_CALIBRATION_LIB_H_
#define FRC971_VISION_CALIBRATION_LIB_H_
#include <cmath>
#include <regex>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/strings/str_format.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/event_loop.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/vision/vision_util_lib.h"

ABSL_DECLARE_FLAG(bool, draw_axes);

namespace frc971::vision {

class IntrinsicsCalibration {
 public:
  IntrinsicsCalibration(aos::EventLoop *event_loop, std::string_view hostname,
                        std::string_view camera_channel,
                        std::string_view camera_id,
                        std::string_view base_intrinsics_file,
                        bool display_undistorted,
                        std::string_view calibration_folder,
                        aos::ExitHandle *exit_handle);

  void HandleCharuco(cv::Mat rgb_image,
                     const aos::monotonic_clock::time_point /*eof*/,
                     std::vector<cv::Vec4i> charuco_ids,
                     std::vector<std::vector<cv::Point2f>> charuco_corners,
                     bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                     std::vector<Eigen::Vector3d> tvecs_eigen);

  void MaybeCalibrate();

  // Expose CharucoExtractor for testing purposes
  CharucoExtractor &GetCharucoExtractor() { return charuco_extractor_; };

  static aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
  BuildCalibration(cv::Mat camera_matrix, cv::Mat dist_coeffs,
                   aos::realtime_clock::time_point realtime_now,
                   std::string_view cpu_type, uint16_t cpu_number,
                   std::string_view camera_channel, std::string_view camera_id,
                   uint16_t team_number, double reprojection_error);

  // Return how many captures we've made so far
  int NumCaptures() const { return all_charuco_ids_.size(); };

 private:
  static constexpr double kDeltaRThreshold = M_PI / 6.0;
  static constexpr double kDeltaTThreshold = 0.3;

  static constexpr double kFrameDeltaRLimit = M_PI / 60;
  static constexpr double kFrameDeltaTLimit = 0.01;

  std::string hostname_;
  const std::optional<std::string_view> cpu_type_;
  const std::optional<uint16_t> cpu_number_;
  const std::string camera_channel_;
  const std::string camera_id_;

  std::vector<std::vector<int>> all_charuco_ids_;
  std::vector<std::vector<cv::Point2f>> all_charuco_corners_;

  // Inverses of the board location in camera frame, for computing deltas later
  Eigen::Affine3d prev_H_board_camera_;
  Eigen::Affine3d last_frame_H_board_camera_;

  // Camera intrinsics that we will use to bootstrap the intrinsics estimation
  // here. We make use of the intrinsics in this calibration to allow us to
  // estimate the relative pose of the charuco board and then identify how much
  // the board is moving.
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      base_intrinsics_;

  CharucoExtractor charuco_extractor_;

  ImageCallback image_callback_;
  cv::Size image_size_;
  // Image used to visualize collected points as they come in
  cv::Mat point_viz_image_;

  const bool display_undistorted_;
  const std::string calibration_folder_;
  aos::ExitHandle *exit_handle_;

  bool exit_collection_;
};

}  // namespace frc971::vision
#endif  // FRC971_VISION_CALIBRATION_LIB_H_
