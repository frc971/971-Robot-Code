#ifndef Y2020_VISION_CHARUCO_LIB_H_
#define Y2020_VISION_CHARUCO_LIB_H_

#include <functional>
#include <string_view>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"

namespace frc971 {
namespace vision {

// Class to find extrinsics for a specified pi's camera using the provided
// training data.
class CameraCalibration {
 public:
  CameraCalibration(const absl::Span<const uint8_t> training_data_bfbs,
                    std::string_view pi);

  // Intrinsics for the located camera.
  cv::Mat CameraIntrinsics() const;
  Eigen::Matrix3d CameraIntrinsicsEigen() const;

  // Distortion coefficients for the located camera.
  cv::Mat CameraDistCoeffs() const;

 private:
  // Finds the camera specific calibration flatbuffer.
  const sift::CameraCalibration *FindCameraCalibration(
      const sift::TrainingData *const training_data, std::string_view pi) const;

  // Pointer to this camera's calibration parameters.
  const sift::CameraCalibration *camera_calibration_;
};

// Class to call a function with a cv::Mat and age when an image shows up on the
// provided channel.  This hides all the conversions and wrangling needed to
// view the image.
class ImageCallback {
 public:
  ImageCallback(
      aos::EventLoop *event_loop, std::string_view channel,
      std::function<void(cv::Mat, aos::monotonic_clock::time_point)> &&fn);

 private:
  aos::EventLoop *event_loop_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> server_fetcher_;
  const aos::Node *source_node_;
  std::function<void(cv::Mat, aos::monotonic_clock::time_point)> handle_image_;
};

// Class which calls a callback each time an image arrives with the information
// extracted from it.
class CharucoExtractor {
 public:
  // The callback takes the following arguments:
  //   cv::Mat -> image with overlays drawn on it.
  //   monotonic_clock::time_point -> Time on this node when this image was
  //                                  captured.
  //   std::vector<int> -> charuco_ids
  //   std::vector<cv::Point2f> -> charuco_corners
  //   bool -> true if rvec/tvec is valid.
  //   Eigen::Vector3d -> rvec
  //   Eigen::Vector3d -> tvec
  CharucoExtractor(
      aos::EventLoop *event_loop, std::string_view pi,
      std::function<void(cv::Mat, aos::monotonic_clock::time_point,
                         std::vector<int>, std::vector<cv::Point2f>, bool,
                         Eigen::Vector3d, Eigen::Vector3d)> &&fn);

  // Returns the aruco dictionary in use.
  cv::Ptr<cv::aruco::Dictionary> dictionary() const { return dictionary_; }
  // Returns the aruco board in use.
  cv::Ptr<cv::aruco::CharucoBoard> board() const { return board_; }

  // Returns the camera matrix for this camera.
  const cv::Mat camera_matrix() const { return camera_matrix_; }
  // Returns the distortion coefficients for this camera.
  const cv::Mat dist_coeffs() const { return dist_coeffs_; }

 private:
  // Handles the image by detecting the charuco board in it.
  void HandleImage(cv::Mat rgb_image, aos::monotonic_clock::time_point eof);

  aos::EventLoop *event_loop_;
  CameraCalibration calibration_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;

  const cv::Mat camera_matrix_;
  const Eigen::Matrix3d eigen_camera_matrix_;
  const cv::Mat dist_coeffs_;

  const std::optional<uint16_t> pi_number_;

  ImageCallback image_callback_;

  // Function to call.
  std::function<void(cv::Mat, aos::monotonic_clock::time_point,
                     std::vector<int>, std::vector<cv::Point2f>, bool,
                     Eigen::Vector3d, Eigen::Vector3d)>
      handle_charuco_;
};

}  // namespace vision
}  // namespace frc971

#endif  // Y2020_VISION_CHARUCO_LIB_H_
