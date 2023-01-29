#ifndef Y2020_VISION_CHARUCO_LIB_H_
#define Y2020_VISION_CHARUCO_LIB_H_

#include <functional>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <string_view>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/types/span.h"
#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"

DECLARE_bool(visualize);

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

// Helper class to call a function with a cv::Mat and age when an image shows up
// on the provided channel.  This hides all the conversions and wrangling needed
// to view the image.
// Can connect this with HandleImage function from CharucoExtrator for
// full-service callback functionality
class ImageCallback {
 public:
  enum class Format {
    YUYV2 = 0,
    BGR = 1,
    GRAYSCALE = 2,
  };
  ImageCallback(aos::EventLoop *event_loop, std::string_view channel,
                std::function<void(cv::Mat, aos::monotonic_clock::time_point)>
                    &&handle_image_fn);

  void set_format(Format format) { format_ = format; }

 private:
  void DisableTracing();

  aos::EventLoop *event_loop_;
  aos::Fetcher<aos::message_bridge::ServerStatistics> server_fetcher_;
  const aos::Node *source_node_;
  std::function<void(cv::Mat, aos::monotonic_clock::time_point)> handle_image_;
  aos::TimerHandler *timer_fn_;

  bool disabling_ = false;

  aos::Ftrace ftrace_;

  Format format_ = Format::BGR;
};

// Types of targets that a CharucoExtractor can detect in images
enum class TargetType : uint8_t {
  kAruco = 0,
  kCharuco = 1,
  kCharucoDiamond = 2
};

// Class which calls a callback each time an image arrives with the information
// extracted from it.
class CharucoExtractor {
 public:
  // The callback takes the following arguments:
  //   cv::Mat -> image with overlays drawn on it.
  //   monotonic_clock::time_point -> Time on this node when this image was
  //                                  captured.
  //   std::vector<Vec4i> -> target ids (aruco/april in first slot of Vec4i)
  // NOTE: We use Vec4i since that stores the ids for the charuco diamond target
  //   std::vector<std::vector<cv::Point2f>> -> charuco_corners
  //   bool -> true if rvec/tvec is valid.
  //   std::vector<Eigen::Vector3d> -> rvec
  //   std::vector<Eigen::Vector3d> -> tvec
  // NOTE: we return as a vector since all but charuco boards could have
  // multiple targets in an image; for charuco boards, there should be just one
  // element
  CharucoExtractor(
      aos::EventLoop *event_loop, std::string_view pi, TargetType target_type,
      std::string_view image_channel,
      std::function<void(cv::Mat, aos::monotonic_clock::time_point,
                         std::vector<cv::Vec4i>,
                         std::vector<std::vector<cv::Point2f>>, bool,
                         std::vector<Eigen::Vector3d>,
                         std::vector<Eigen::Vector3d>)> &&handle_charuco_fn);

  // Handles the image by detecting the charuco board in it.
  void HandleImage(cv::Mat rgb_image, aos::monotonic_clock::time_point eof);

  // Returns the aruco dictionary in use.
  cv::Ptr<cv::aruco::Dictionary> dictionary() const { return dictionary_; }
  // Returns the aruco board in use.
  cv::Ptr<cv::aruco::CharucoBoard> board() const { return board_; }

  // Returns the camera matrix for this camera.
  const cv::Mat camera_matrix() const { return camera_matrix_; }
  // Returns the distortion coefficients for this camera.
  const cv::Mat dist_coeffs() const { return dist_coeffs_; }

 private:
  // Creates the dictionary, board, and other parameters for the appropriate
  // (ch)aruco target
  void SetupTargetData();

  // Draw the axes from the pose(s) on the image
  void DrawTargetPoses(cv::Mat rgb_image, std::vector<cv::Vec3d> rvecs,
                       std::vector<cv::Vec3d> tvecs);

  // Helper function to convert rotation (rvecs) and translation (tvecs) vectors
  // into Eigen vectors and store in corresponding vectors
  void PackPoseResults(std::vector<cv::Vec3d> &rvecs,
                       std::vector<cv::Vec3d> &tvecs,
                       std::vector<Eigen::Vector3d> *rvecs_eigen,
                       std::vector<Eigen::Vector3d> *tvecs_eigen);

  aos::EventLoop *event_loop_;
  CameraCalibration calibration_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;

  // Type of targets to detect
  TargetType target_type_;
  // Channel to listen on for images
  std::string_view image_channel_;

  // Length of a side of the target marker
  double marker_length_;
  // Length of a side of the checkerboard squares (around the marker)
  double square_length_;

  // Intrinsic calibration matrix
  const cv::Mat camera_matrix_;
  // Intrinsic calibration matrix as Eigen::Matrix3d
  const Eigen::Matrix3d eigen_camera_matrix_;
  // Intrinsic distortion coefficients
  const cv::Mat dist_coeffs_;

  // Index number of the raspberry pi
  const std::optional<uint16_t> pi_number_;

  // Function to call.
  std::function<void(
      cv::Mat, aos::monotonic_clock::time_point, std::vector<cv::Vec4i>,
      std::vector<std::vector<cv::Point2f>>, bool, std::vector<Eigen::Vector3d>,
      std::vector<Eigen::Vector3d>)>
      handle_charuco_;
};

}  // namespace vision
}  // namespace frc971

#endif  // Y2020_VISION_CHARUCO_LIB_H_
