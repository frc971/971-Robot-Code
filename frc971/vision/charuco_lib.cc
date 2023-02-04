#include "frc971/vision/charuco_lib.h"

#include <chrono>
#include <functional>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string_view>

#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/vision_generated.h"
#include "glog/logging.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"

DEFINE_string(board_template_path, "",
              "If specified, write an image to the specified path for the "
              "charuco board pattern.");
DEFINE_bool(coarse_pattern, true, "If true, use coarse arucos; else, use fine");
DEFINE_bool(large_board, true, "If true, use the large calibration board.");
DEFINE_uint32(
    min_charucos, 10,
    "The mininum number of aruco targets in charuco board required to match.");
DEFINE_bool(visualize, false, "Whether to visualize the resulting data.");

DEFINE_uint32(disable_delay, 100, "Time after an issue to disable tracing at.");

DECLARE_bool(enable_ftrace);

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
    std::function<void(cv::Mat, monotonic_clock::time_point)> &&handle_image_fn,
    monotonic_clock::duration max_age)
    : event_loop_(event_loop),
      server_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/aos")),
      source_node_(aos::configuration::GetNode(
          event_loop_->configuration(),
          event_loop_->GetChannel<CameraImage>(channel)
              ->source_node()
              ->string_view())),
      handle_image_(std::move(handle_image_fn)),
      timer_fn_(event_loop->AddTimer([this]() { DisableTracing(); })),
      max_age_(max_age) {
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
    if (age > max_age_) {
      if (FLAGS_enable_ftrace) {
        ftrace_.FormatMessage("Too late receiving image, age: %f\n",
                              age_double);
        if (FLAGS_disable_delay > 0) {
          if (!disabling_) {
            timer_fn_->Setup(event_loop_->monotonic_now() +
                             chrono::milliseconds(FLAGS_disable_delay));
            disabling_ = true;
          }
        } else {
          DisableTracing();
        }
      }
      VLOG(2) << "Age: " << age_double << ", getting behind, skipping";
      return;
    }
    // Create color image:
    cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()), CV_8UC2,
                            (void *)image.data()->data());
    const cv::Size image_size(image.cols(), image.rows());
    switch (format_) {
      case Format::GRAYSCALE: {
        ftrace_.FormatMessage("Starting yuyv->greyscale\n");
        cv::Mat gray_image(image_size, CV_8UC3);
        cv::cvtColor(image_color_mat, gray_image, cv::COLOR_YUV2GRAY_YUYV);
        handle_image_(gray_image, eof);
      } break;
      case Format::BGR: {
        cv::Mat rgb_image(image_size, CV_8UC3);
        cv::cvtColor(image_color_mat, rgb_image, cv::COLOR_YUV2BGR_YUYV);
        handle_image_(rgb_image, eof);
      } break;
      case Format::YUYV2: {
        handle_image_(image_color_mat, eof);
      };
    }
  });
}

void ImageCallback::DisableTracing() {
  disabling_ = false;
  ftrace_.TurnOffOrDie();
}

void CharucoExtractor::SetupTargetData() {
  // TODO(Jim): Put correct values here
  marker_length_ = 0.15;
  square_length_ = 0.1651;

  // Only charuco board has a board associated with it
  board_ = static_cast<cv::Ptr<cv::aruco::CharucoBoard>>(NULL);

  if (target_type_ == TargetType::kCharuco ||
      target_type_ == TargetType::kAruco) {
    dictionary_ = cv::aruco::getPredefinedDictionary(
        FLAGS_large_board ? cv::aruco::DICT_5X5_250 : cv::aruco::DICT_6X6_250);

    if (target_type_ == TargetType::kCharuco) {
      LOG(INFO) << "Using " << (FLAGS_large_board ? " large " : " small ")
                << " charuco board with "
                << (FLAGS_coarse_pattern ? "coarse" : "fine") << " pattern";
      board_ =
          (FLAGS_large_board
               ? (FLAGS_coarse_pattern ? cv::aruco::CharucoBoard::create(
                                             12, 9, 0.06, 0.04666, dictionary_)
                                       : cv::aruco::CharucoBoard::create(
                                             25, 18, 0.03, 0.0233, dictionary_))
               : (FLAGS_coarse_pattern ? cv::aruco::CharucoBoard::create(
                                             7, 5, 0.04, 0.025, dictionary_)
                                       // TODO(jim): Need to figure out what
                                       // size is for small board, fine pattern
                                       : cv::aruco::CharucoBoard::create(
                                             7, 5, 0.03, 0.0233, dictionary_)));
      if (!FLAGS_board_template_path.empty()) {
        cv::Mat board_image;
        board_->draw(cv::Size(600, 500), board_image, 10, 1);
        cv::imwrite(FLAGS_board_template_path, board_image);
      }
    }
  } else if (target_type_ == TargetType::kCharucoDiamond) {
    // TODO<Jim>: Measure this
    marker_length_ = 0.15;
    square_length_ = 0.1651;
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  } else {
    // Bail out if it's not a supported target
    LOG(FATAL) << "Target type undefined: "
               << static_cast<uint8_t>(target_type_);
  }
}

void CharucoExtractor::DrawTargetPoses(cv::Mat rgb_image,
                                       std::vector<cv::Vec3d> rvecs,
                                       std::vector<cv::Vec3d> tvecs) {
  const Eigen::Matrix<double, 3, 4> camera_projection =
      Eigen::Matrix<double, 3, 4>::Identity();

  int x_coord = 10;
  int y_coord = 0;
  // draw axis for each marker
  for (uint i = 0; i < rvecs.size(); i++) {
    Eigen::Vector3d rvec_eigen, tvec_eigen;
    cv::cv2eigen(rvecs[i], rvec_eigen);
    cv::cv2eigen(tvecs[i], tvec_eigen);

    Eigen::Quaternion<double> rotation(
        frc971::controls::ToQuaternionFromRotationVector(rvec_eigen));
    Eigen::Translation3d translation(tvec_eigen);

    const Eigen::Affine3d board_to_camera = translation * rotation;

    Eigen::Vector3d result = eigen_camera_matrix_ * camera_projection *
                             board_to_camera * Eigen::Vector3d::Zero();

    // Found that drawAxis hangs if you try to draw with z values too
    // small (trying to draw axes at inifinity)
    // TODO<Jim>: Explore what real thresholds for this should be;
    // likely Don't need to get rid of negative values
    if (result.z() < 0.01) {
      LOG(INFO) << "Skipping, due to z value too small: " << result.z();
    } else {
      result /= result.z();
      if (target_type_ == TargetType::kCharuco) {
        cv::aruco::drawAxis(rgb_image, camera_matrix_, dist_coeffs_, rvecs[i],
                            tvecs[i], 0.1);
      } else {
        cv::drawFrameAxes(rgb_image, camera_matrix_, dist_coeffs_, rvecs[i],
                          tvecs[i], 0.1);
      }
    }
    std::stringstream ss;
    ss << "tvec[" << i << "] = " << tvecs[i];
    y_coord += 25;
    cv::putText(rgb_image, ss.str(), cv::Point(x_coord, y_coord),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
    ss.str("");
    ss << "rvec[" << i << "] = " << rvecs[i];
    y_coord += 25;
    cv::putText(rgb_image, ss.str(), cv::Point(x_coord, y_coord),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
  }
}

void CharucoExtractor::PackPoseResults(
    std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs,
    std::vector<Eigen::Vector3d> *rvecs_eigen,
    std::vector<Eigen::Vector3d> *tvecs_eigen) {
  for (cv::Vec3d rvec : rvecs) {
    Eigen::Vector3d rvec_eigen = Eigen::Vector3d::Zero();
    cv::cv2eigen(rvec, rvec_eigen);
    rvecs_eigen->emplace_back(rvec_eigen);
  }

  for (cv::Vec3d tvec : tvecs) {
    Eigen::Vector3d tvec_eigen = Eigen::Vector3d::Zero();
    cv::cv2eigen(tvec, tvec_eigen);
    tvecs_eigen->emplace_back(tvec_eigen);
  }
}

CharucoExtractor::CharucoExtractor(
    aos::EventLoop *event_loop, std::string_view pi, TargetType target_type,
    std::string_view image_channel,
    std::function<void(cv::Mat, monotonic_clock::time_point,
                       std::vector<cv::Vec4i>,
                       std::vector<std::vector<cv::Point2f>>, bool,
                       std::vector<Eigen::Vector3d>,
                       std::vector<Eigen::Vector3d>)> &&handle_charuco_fn)
    : event_loop_(event_loop),
      calibration_(SiftTrainingData(), pi),
      target_type_(target_type),
      image_channel_(image_channel),
      camera_matrix_(calibration_.CameraIntrinsics()),
      eigen_camera_matrix_(calibration_.CameraIntrinsicsEigen()),
      dist_coeffs_(calibration_.CameraDistCoeffs()),
      pi_number_(aos::network::ParsePiNumber(pi)),
      handle_charuco_(std::move(handle_charuco_fn)) {
  SetupTargetData();

  LOG(INFO) << "Camera matrix " << camera_matrix_;
  LOG(INFO) << "Distortion Coefficients " << dist_coeffs_;

  CHECK(pi_number_) << ": Invalid pi number " << pi
                    << ", failed to parse pi number";

  LOG(INFO) << "Connecting to channel " << image_channel_;
}

void CharucoExtractor::HandleImage(cv::Mat rgb_image,
                                   const monotonic_clock::time_point eof) {
  const double age_double =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          event_loop_->monotonic_now() - eof)
          .count();

  // Set up the variables we'll use in the callback function
  bool valid = false;
  // Return a list of poses; for Charuco Board there will be just one
  std::vector<Eigen::Vector3d> rvecs_eigen;
  std::vector<Eigen::Vector3d> tvecs_eigen;

  // ids and corners for initial aruco marker detections
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;

  // ids and corners for final, refined board / marker detections
  // Using Vec4i type since it supports Charuco Diamonds
  // And overloading it using 1st int in Vec4i for others target types
  std::vector<cv::Vec4i> result_ids;
  std::vector<std::vector<cv::Point2f>> result_corners;

  // Do initial marker detection; this is the same for all target types
  cv::aruco::detectMarkers(rgb_image, dictionary_, marker_corners, marker_ids);
  cv::aruco::drawDetectedMarkers(rgb_image, marker_corners, marker_ids);

  VLOG(2) << "Handle Image, with target type = "
          << static_cast<uint8_t>(target_type_) << " and " << marker_ids.size()
          << " markers detected initially";

  if (marker_ids.size() == 0) {
    VLOG(2) << "Didn't find any markers";
  } else {
    if (target_type_ == TargetType::kCharuco) {
      std::vector<int> charuco_ids;
      std::vector<cv::Point2f> charuco_corners;

      // If enough aruco markers detected for the Charuco board
      if (marker_ids.size() >= FLAGS_min_charucos) {
        // Run everything twice, once with the calibration, and once
        // without. This lets us both collect data to calibrate the
        // intrinsics of the camera (to determine the intrinsics from
        // multiple samples), and also to use data from a previous/stored
        // calibration to determine a more accurate pose in real time (used
        // for extrinsics calibration)
        cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids,
                                             rgb_image, board_, charuco_corners,
                                             charuco_ids);

        std::vector<cv::Point2f> charuco_corners_with_calibration;
        std::vector<int> charuco_ids_with_calibration;

        // This call uses a previous intrinsic calibration to get more
        // accurate marker locations, for a better pose estimate
        cv::aruco::interpolateCornersCharuco(
            marker_corners, marker_ids, rgb_image, board_,
            charuco_corners_with_calibration, charuco_ids_with_calibration,
            camera_matrix_, dist_coeffs_);

        if (charuco_ids.size() >= FLAGS_min_charucos) {
          cv::aruco::drawDetectedCornersCharuco(
              rgb_image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));

          cv::Vec3d rvec, tvec;
          valid = cv::aruco::estimatePoseCharucoBoard(
              charuco_corners_with_calibration, charuco_ids_with_calibration,
              board_, camera_matrix_, dist_coeffs_, rvec, tvec);

          // if charuco pose is valid, return pose, with ids and corners
          if (valid) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            rvecs.emplace_back(rvec);
            tvecs.emplace_back(tvec);
            DrawTargetPoses(rgb_image, rvecs, tvecs);

            PackPoseResults(rvecs, tvecs, &rvecs_eigen, &tvecs_eigen);
            // Store the corners without calibration, since we use them to
            // do calibration
            result_corners.emplace_back(charuco_corners);
            for (auto id : charuco_ids) {
              result_ids.emplace_back(cv::Vec4i{id, 0, 0, 0});
            }
          } else {
            VLOG(2) << "Age: " << age_double << ", invalid charuco board pose";
          }
        } else {
          VLOG(2) << "Age: " << age_double << ", not enough charuco IDs, got "
                  << charuco_ids.size() << ", needed " << FLAGS_min_charucos;
        }
      } else {
        VLOG(2) << "Age: " << age_double
                << ", not enough marker IDs for charuco board, got "
                << marker_ids.size() << ", needed " << FLAGS_min_charucos;
      }
    } else if (target_type_ == TargetType::kAruco) {
      // estimate pose for arucos doesn't return valid, so marking true
      valid = true;
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(marker_corners, square_length_,
                                           camera_matrix_, dist_coeffs_, rvecs,
                                           tvecs);
      DrawTargetPoses(rgb_image, rvecs, tvecs);

      PackPoseResults(rvecs, tvecs, &rvecs_eigen, &tvecs_eigen);
      for (uint i = 0; i < marker_ids.size(); i++) {
        result_ids.emplace_back(cv::Vec4i{marker_ids[i], 0, 0, 0});
      }
      result_corners = marker_corners;
    } else if (target_type_ == TargetType::kCharucoDiamond) {
      // Extract the diamonds associated with the markers
      std::vector<cv::Vec4i> diamond_ids;
      std::vector<std::vector<cv::Point2f>> diamond_corners;
      cv::aruco::detectCharucoDiamond(rgb_image, marker_corners, marker_ids,
                                      square_length_ / marker_length_,
                                      diamond_corners, diamond_ids);

      // Check to see if we found any diamond targets
      if (diamond_ids.size() > 0) {
        cv::aruco::drawDetectedDiamonds(rgb_image, diamond_corners,
                                        diamond_ids);

        // estimate pose for diamonds doesn't return valid, so marking true
        valid = true;
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(diamond_corners, square_length_,
                                             camera_matrix_, dist_coeffs_,
                                             rvecs, tvecs);
        DrawTargetPoses(rgb_image, rvecs, tvecs);

        PackPoseResults(rvecs, tvecs, &rvecs_eigen, &tvecs_eigen);
        result_ids = diamond_ids;
        result_corners = diamond_corners;
      } else {
        VLOG(2) << "Found aruco markers, but no charuco diamond targets";
      }
    } else {
      LOG(FATAL) << "Unknown target type: "
                 << static_cast<uint8_t>(target_type_);
    }
  }

  handle_charuco_(rgb_image, eof, result_ids, result_corners, valid,
                  rvecs_eigen, tvecs_eigen);
}

}  // namespace vision
}  // namespace frc971
