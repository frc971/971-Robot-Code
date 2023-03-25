#ifndef Y2020_VISION_CHARUCO_LIB_H_
#define Y2020_VISION_CHARUCO_LIB_H_

#include <functional>
#include <string_view>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/types/span.h"
#include "external/com_github_foxglove_schemas/ImageAnnotations_generated.h"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_server_generated.h"
#include "frc971/vision/calibration_generated.h"

DECLARE_bool(visualize);

namespace frc971 {
namespace vision {

// Class to find extrinsics for a specified pi's camera using the provided
// training data.
class CameraCalibration {
 public:
  CameraCalibration(const calibration::CameraCalibration *calibration);

  // Intrinsics for the located camera.
  cv::Mat CameraIntrinsics() const { return intrinsics_; }
  Eigen::Matrix3d CameraIntrinsicsEigen() const { return intrinsics_eigen_; }

  // Distortion coefficients for the located camera.
  cv::Mat CameraDistCoeffs() const { return dist_coeffs_; }

 private:
  const cv::Mat intrinsics_;
  const Eigen::Matrix3d intrinsics_eigen_;
  const cv::Mat dist_coeffs_;
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

  // `max_age` is the age to start dropping frames at
  ImageCallback(
      aos::EventLoop *event_loop, std::string_view channel,
      std::function<void(cv::Mat, aos::monotonic_clock::time_point)>
          &&handle_image_fn,
      aos::monotonic_clock::duration max_age = std::chrono::milliseconds(100));

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

  aos::monotonic_clock::duration max_age_;
};

// Types of targets that a CharucoExtractor can detect in images
enum class TargetType : uint8_t {
  kAruco = 0,
  kCharuco = 1,
  kCharucoDiamond = 2,
  kAprilTag = 3
};

TargetType TargetTypeFromString(std::string_view str);
std::ostream &operator<<(std::ostream &os, TargetType target_type);

// Class which calls a callback each time an image arrives with the information
// extracted from it.
class CharucoExtractor {
 public:
  // Setting up a constructor that doesn't require an event_loop, so we can call
  // and get results back from ProcessImage directly
  CharucoExtractor(const calibration::CameraCalibration *calibration,
                   const TargetType target_type);

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
      aos::EventLoop *event_loop,
      const calibration::CameraCalibration *calibration, TargetType target_type,
      std::string_view image_channel,
      std::function<void(cv::Mat, aos::monotonic_clock::time_point,
                         std::vector<cv::Vec4i>,
                         std::vector<std::vector<cv::Point2f>>, bool,
                         std::vector<Eigen::Vector3d>,
                         std::vector<Eigen::Vector3d>)> &&handle_charuco_fn);

  // Handles the image by detecting the charuco board in it.
  void HandleImage(cv::Mat rgb_image,
                   const aos::monotonic_clock::time_point eof);

  void ProcessImage(cv::Mat rgb_image,
                    const aos::monotonic_clock::time_point eof,
                    const aos::monotonic_clock::time_point current_time,
                    std::vector<cv::Vec4i> &result_ids,
                    std::vector<std::vector<cv::Point2f>> &result_corners,
                    bool &valid, std::vector<Eigen::Vector3d> &rvecs_eigen,
                    std::vector<Eigen::Vector3d> &tvecs_eigen);

  // Returns the aruco dictionary in use.
  cv::Ptr<cv::aruco::Dictionary> dictionary() const { return dictionary_; }
  // Returns the aruco board in use.
  cv::Ptr<cv::aruco::CharucoBoard> board() const { return board_; }

  // Returns the camera matrix for this camera.
  const cv::Mat camera_matrix() const {
    return calibration_.CameraIntrinsics();
  }
  // Returns the distortion coefficients for this camera.
  const cv::Mat dist_coeffs() const { return calibration_.CameraDistCoeffs(); }

 private:
  // Creates the dictionary, board, and other parameters for the appropriate
  // (ch)aruco target
  void SetupTargetData();

  // Draw the axes from the pose(s) on the image
  void DrawTargetPoses(cv::Mat rgb_image, std::vector<cv::Vec3d> rvecs,
                       std::vector<cv::Vec3d> tvecs);

  // Helper function to convert rotation (rvecs) and translation (tvecs)
  // vectors into Eigen vectors and store in corresponding vectors
  void PackPoseResults(std::vector<cv::Vec3d> &rvecs,
                       std::vector<cv::Vec3d> &tvecs,
                       std::vector<Eigen::Vector3d> *rvecs_eigen,
                       std::vector<Eigen::Vector3d> *tvecs_eigen);

  aos::EventLoop *event_loop_;

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

  CameraCalibration calibration_;

  // Function to call.
  std::function<void(
      cv::Mat, aos::monotonic_clock::time_point, std::vector<cv::Vec4i>,
      std::vector<std::vector<cv::Point2f>>, bool, std::vector<Eigen::Vector3d>,
      std::vector<Eigen::Vector3d>)>
      handle_charuco_;
};

// Puts the provided charuco corners into a foxglove ImageAnnotation type for
// visualization purposes.
flatbuffers::Offset<foxglove::ImageAnnotations> BuildAnnotations(
    flatbuffers::FlatBufferBuilder *fbb,
    const aos::monotonic_clock::time_point monotonic_now,
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<double> rgba_color = std::vector<double>{0.0, 1.0, 0.0,
                                                               1.0},
    const double thickness = 5,
    const foxglove::PointsAnnotationType line_type =
        foxglove::PointsAnnotationType::POINTS);

// Creates a PointsAnnotation to build up ImageAnnotations with different
// types
flatbuffers::Offset<foxglove::PointsAnnotation> BuildPointsAnnotation(
    flatbuffers::FlatBufferBuilder *fbb,
    const aos::monotonic_clock::time_point monotonic_now,
    const std::vector<cv::Point2f> &corners,
    const std::vector<double> rgba_color = std::vector<double>{0.0, 1.0, 0.0,
                                                               1.0},
    const double thickness = 5,
    const foxglove::PointsAnnotationType line_type =
        foxglove::PointsAnnotationType::POINTS);

}  // namespace vision
}  // namespace frc971

#endif  // Y2020_VISION_CHARUCO_LIB_H_
