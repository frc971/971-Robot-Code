#include "y2020/vision/charuco_lib.h"

#include <chrono>
#include <functional>
#include <string_view>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/vision_generated.h"
#include "glog/logging.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"

DEFINE_uint32(min_targets, 10,
              "The mininum number of targets required to match.");
DEFINE_bool(large_board, true, "If true, use the large calibration board.");
DEFINE_bool(coarse_pattern, true, "If true, use coarse arucos; else, use fine");
DEFINE_string(board_template_path, "",
              "If specified, write an image to the specified path for the "
              "charuco board pattern.");

namespace frc971 {
namespace vision {
namespace chrono = std::chrono;
using aos::monotonic_clock;

CameraCalibration::CameraCalibration(
    const absl::Span<const uint8_t> training_data_bfbs, std::string_view pi) {
  const aos::FlatbufferSpan<sift::TrainingData> training_data(
      training_data_bfbs);
  CHECK(training_data.Verify());
  camera_calibration_ = FindCameraCalibration(&training_data.message(), pi);
}

cv::Mat CameraCalibration::CameraIntrinsics() const {
  const cv::Mat result(3, 3, CV_32F,
                       const_cast<void *>(static_cast<const void *>(
                           camera_calibration_->intrinsics()->data())));
  CHECK_EQ(result.total(), camera_calibration_->intrinsics()->size());
  return result;
}

Eigen::Matrix3d CameraCalibration::CameraIntrinsicsEigen() const {
  cv::Mat camera_intrinsics = CameraIntrinsics();
  Eigen::Matrix3d result;
  cv::cv2eigen(camera_intrinsics, result);
  return result;
}

cv::Mat CameraCalibration::CameraDistCoeffs() const {
  const cv::Mat result(5, 1, CV_32F,
                       const_cast<void *>(static_cast<const void *>(
                           camera_calibration_->dist_coeffs()->data())));
  CHECK_EQ(result.total(), camera_calibration_->dist_coeffs()->size());
  return result;
}

const sift::CameraCalibration *CameraCalibration::FindCameraCalibration(
    const sift::TrainingData *const training_data, std::string_view pi) const {
  std::optional<uint16_t> pi_number = aos::network::ParsePiNumber(pi);
  std::optional<uint16_t> team_number =
      aos::network::team_number_internal::ParsePiTeamNumber(pi);
  CHECK(pi_number);
  CHECK(team_number);
  const std::string node_name = absl::StrFormat("pi%d", pi_number.value());
  LOG(INFO) << "Looking for node name " << node_name << " team number "
            << team_number.value();
  for (const sift::CameraCalibration *candidate :
       *training_data->camera_calibrations()) {
    if (candidate->node_name()->string_view() != node_name) {
      continue;
    }
    if (candidate->team_number() != team_number.value()) {
      continue;
    }
    return candidate;
  }
  LOG(FATAL) << ": Failed to find camera calibration for " << node_name
             << " on " << team_number.value();
}

ImageCallback::ImageCallback(
    aos::EventLoop *event_loop, std::string_view channel,
    std::function<void(cv::Mat, monotonic_clock::time_point)> &&fn)
    : event_loop_(event_loop),
      server_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/aos")),
      source_node_(aos::configuration::GetNode(
          event_loop_->configuration(),
          event_loop_->GetChannel<CameraImage>(channel)
              ->source_node()
              ->string_view())),
      handle_image_(std::move(fn)) {
  event_loop_->MakeWatcher(channel, [this](const CameraImage &image) {
    const monotonic_clock::time_point eof_source_node =
        monotonic_clock::time_point(
            chrono::nanoseconds(image.monotonic_timestamp_ns()));
    chrono::nanoseconds offset{0};
    if (source_node_ != event_loop_->node()) {
      server_fetcher_.Fetch();
      if (!server_fetcher_.get()) {
        return;
      }

      // If we are viewing this image from another node, convert to our
      // monotonic clock.
      const aos::message_bridge::ServerConnection *server_connection = nullptr;

      for (const aos::message_bridge::ServerConnection *connection :
           *server_fetcher_->connections()) {
        CHECK(connection->has_node());
        if (connection->node()->name()->string_view() ==
            source_node_->name()->string_view()) {
          server_connection = connection;
          break;
        }
      }

      CHECK(server_connection != nullptr) << ": Failed to find client";
      if (!server_connection->has_monotonic_offset()) {
        VLOG(1) << "No offset yet.";
        return;
      }
      offset = chrono::nanoseconds(server_connection->monotonic_offset());
    }

    const monotonic_clock::time_point eof = eof_source_node - offset;

    const monotonic_clock::duration age = event_loop_->monotonic_now() - eof;
    const double age_double =
        std::chrono::duration_cast<std::chrono::duration<double>>(age).count();
    if (age > std::chrono::milliseconds(100)) {
      LOG(INFO) << "Age: " << age_double << ", getting behind, skipping";
      return;
    }
    // Create color image:
    cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                            (void *)image.data()->data());
    const cv::Size image_size(image.cols(), image.rows());
    cv::Mat rgb_image(image_size, CV_8UC3);
    cv::cvtColor(image_color_mat, rgb_image, cv::COLOR_YUV2BGR_YUYV);
    handle_image_(rgb_image, eof);
  });
}

CharucoExtractor::CharucoExtractor(
    aos::EventLoop *event_loop, std::string_view pi,
    std::function<void(cv::Mat, monotonic_clock::time_point, std::vector<int>,
                       std::vector<cv::Point2f>, bool, Eigen::Vector3d,
                       Eigen::Vector3d)> &&fn)
    : event_loop_(event_loop),
      calibration_(SiftTrainingData(), pi),
      dictionary_(cv::aruco::getPredefinedDictionary(
          FLAGS_large_board ? cv::aruco::DICT_5X5_250
                            : cv::aruco::DICT_6X6_250)),
      board_(
          FLAGS_large_board
              ? (FLAGS_coarse_pattern ? cv::aruco::CharucoBoard::create(
                                            12, 9, 0.06, 0.04666, dictionary_)
                                      : cv::aruco::CharucoBoard::create(
                                            25, 18, 0.03, 0.0233, dictionary_))
              : (FLAGS_coarse_pattern ? cv::aruco::CharucoBoard::create(
                                            7, 5, 0.04, 0.025, dictionary_)
                                      // TODO(jim): Need to figure out what size
                                      // is for small board, fine pattern
                                      : cv::aruco::CharucoBoard::create(
                                            7, 5, 0.03, 0.0233, dictionary_))),
      camera_matrix_(calibration_.CameraIntrinsics()),
      eigen_camera_matrix_(calibration_.CameraIntrinsicsEigen()),
      dist_coeffs_(calibration_.CameraDistCoeffs()),
      pi_number_(aos::network::ParsePiNumber(pi)),
      image_callback_(
          event_loop,
          absl::StrCat("/pi", std::to_string(pi_number_.value()), "/camera"),
          [this](cv::Mat rgb_image, const monotonic_clock::time_point eof) {
            HandleImage(rgb_image, eof);
          }),
      handle_charuco_(std::move(fn)) {
  LOG(INFO) << "Using " << (FLAGS_large_board ? "large" : "small")
            << " board with " << (FLAGS_coarse_pattern ? "coarse" : "fine")
            << " pattern";
  if (!FLAGS_board_template_path.empty()) {
    cv::Mat board_image;
    board_->draw(cv::Size(600, 500), board_image, 10, 1);
    cv::imwrite(FLAGS_board_template_path, board_image);
  }

  LOG(INFO) << "Camera matrix " << camera_matrix_;
  LOG(INFO) << "Distortion Coefficients " << dist_coeffs_;

  CHECK(pi_number_) << ": Invalid pi number " << pi
                    << ", failed to parse pi number";

  LOG(INFO) << "Connecting to channel /pi" << pi_number_.value() << "/camera";
}

void CharucoExtractor::HandleImage(cv::Mat rgb_image,
                                   const monotonic_clock::time_point eof) {
  const double age_double =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          event_loop_->monotonic_now() - eof)
          .count();
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;

  cv::aruco::detectMarkers(rgb_image, board_->dictionary, marker_corners,
                           marker_ids);

  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  bool valid = false;
  Eigen::Vector3d rvec_eigen = Eigen::Vector3d::Zero();
  Eigen::Vector3d tvec_eigen = Eigen::Vector3d::Zero();

  // If at least one marker detected
  if (marker_ids.size() >= FLAGS_min_targets) {
    // Run everything twice, once with the calibration, and once
    // without. This lets us both calibrate, and also print out the pose
    // real time with the previous calibration.
    cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, rgb_image,
                                         board_, charuco_corners, charuco_ids);

    std::vector<cv::Point2f> charuco_corners_with_calibration;
    std::vector<int> charuco_ids_with_calibration;

    cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, rgb_image, board_,
        charuco_corners_with_calibration, charuco_ids_with_calibration,
        camera_matrix_, dist_coeffs_);

    cv::aruco::drawDetectedMarkers(rgb_image, marker_corners, marker_ids);

    if (charuco_ids.size() >= FLAGS_min_targets) {
      cv::aruco::drawDetectedCornersCharuco(rgb_image, charuco_corners,
                                            charuco_ids, cv::Scalar(255, 0, 0));

      cv::Vec3d rvec, tvec;
      valid = cv::aruco::estimatePoseCharucoBoard(
          charuco_corners_with_calibration, charuco_ids_with_calibration,
          board_, camera_matrix_, dist_coeffs_, rvec, tvec);

      // if charuco pose is valid
      if (valid) {
        cv::cv2eigen(rvec, rvec_eigen);
        cv::cv2eigen(tvec, tvec_eigen);

        Eigen::Quaternion<double> rotation(
            frc971::controls::ToQuaternionFromRotationVector(rvec_eigen));
        Eigen::Translation3d translation(tvec_eigen);

        const Eigen::Affine3d board_to_camera = translation * rotation;

        Eigen::Matrix<double, 3, 4> camera_projection =
            Eigen::Matrix<double, 3, 4>::Identity();
        Eigen::Vector3d result = eigen_camera_matrix_ * camera_projection *
                                 board_to_camera * Eigen::Vector3d::Zero();

        result /= result.z();
        cv::circle(rgb_image, cv::Point(result.x(), result.y()), 4,
                   cv::Scalar(255, 255, 255), 0, cv::LINE_8);

        cv::aruco::drawAxis(rgb_image, camera_matrix_, dist_coeffs_, rvec, tvec,
                            0.1);
      } else {
        LOG(INFO) << "Age: " << age_double << ", invalid pose";
      }
    } else {
      LOG(INFO) << "Age: " << age_double << ", not enough charuco IDs, got "
                << charuco_ids.size() << ", needed " << FLAGS_min_targets;
    }
  } else {
    LOG(INFO) << "Age: " << age_double << ", not enough marker IDs, got "
              << marker_ids.size() << ", needed " << FLAGS_min_targets;
    cv::aruco::drawDetectedMarkers(rgb_image, marker_corners, marker_ids);
  }

  handle_charuco_(rgb_image, eof, charuco_ids, charuco_corners, valid,
                  rvec_eigen, tvec_eigen);
}

}  // namespace vision
}  // namespace frc971
