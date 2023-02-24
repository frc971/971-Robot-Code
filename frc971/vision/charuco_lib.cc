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

DEFINE_string(board_template_path, "",
              "If specified, write an image to the specified path for the "
              "charuco board pattern.");
DEFINE_bool(coarse_pattern, true, "If true, use coarse arucos; else, use fine");
DEFINE_uint32(gray_threshold, 0,
              "If > 0, threshold image based on this grayscale value");
DEFINE_bool(large_board, true, "If true, use the large calibration board.");
DEFINE_uint32(
    min_charucos, 10,
    "The mininum number of aruco targets in charuco board required to match.");
DEFINE_bool(visualize, false, "Whether to visualize the resulting data.");
DEFINE_bool(
    draw_axes, false,
    "Whether to draw axes on the resulting data-- warning, may cause crashes.");

DEFINE_uint32(disable_delay, 100, "Time after an issue to disable tracing at.");

DECLARE_bool(enable_ftrace);

namespace frc971 {
namespace vision {
namespace chrono = std::chrono;
using aos::monotonic_clock;

CameraCalibration::CameraCalibration(
    const calibration::CameraCalibration *calibration)
    : intrinsics_([calibration]() {
        const cv::Mat result(3, 3, CV_32F,
                             const_cast<void *>(static_cast<const void *>(
                                 calibration->intrinsics()->data())));
        CHECK_EQ(result.total(), calibration->intrinsics()->size());
        return result;
      }()),
      intrinsics_eigen_([this]() {
        cv::Mat camera_intrinsics = intrinsics_;
        Eigen::Matrix3d result;
        cv::cv2eigen(camera_intrinsics, result);
        return result;
      }()),
      dist_coeffs_([calibration]() {
        const cv::Mat result(5, 1, CV_32F,
                             const_cast<void *>(static_cast<const void *>(
                                 calibration->dist_coeffs()->data())));
        CHECK_EQ(result.total(), calibration->dist_coeffs()->size());
        return result;
      }()) {}

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
  marker_length_ = 0.146;
  square_length_ = 0.2;

  // Only charuco board has a board associated with it
  board_ = static_cast<cv::Ptr<cv::aruco::CharucoBoard>>(NULL);

  if (target_type_ == TargetType::kCharuco ||
      target_type_ == TargetType::kAruco) {
    dictionary_ = cv::aruco::getPredefinedDictionary(
        FLAGS_large_board ? cv::aruco::DICT_5X5_250 : cv::aruco::DICT_6X6_250);

    if (target_type_ == TargetType::kCharuco) {
      LOG(INFO) << "Using " << (FLAGS_large_board ? "large" : "small")
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
    marker_length_ = 0.15;
    square_length_ = 0.2;
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

    Eigen::Vector3d result = calibration_.CameraIntrinsicsEigen() *
                             camera_projection * board_to_camera *
                             Eigen::Vector3d::Zero();

    // Found that drawAxis hangs if you try to draw with z values too
    // small (trying to draw axes at inifinity)
    // TODO<Jim>: Either track this down or reimplement drawAxes
    if (result.z() < 0.01) {
      LOG(INFO) << "Skipping, due to z value too small: " << result.z();
    } else if (FLAGS_draw_axes == true) {
      result /= result.z();
      if (target_type_ == TargetType::kCharuco) {
        cv::aruco::drawAxis(rgb_image, calibration_.CameraIntrinsics(),
                            calibration_.CameraDistCoeffs(), rvecs[i], tvecs[i],
                            0.1);
      } else {
        cv::drawFrameAxes(rgb_image, calibration_.CameraIntrinsics(),
                          calibration_.CameraDistCoeffs(), rvecs[i], tvecs[i],
                          0.1);
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
    aos::EventLoop *event_loop,
    const calibration::CameraCalibration *calibration, TargetType target_type,
    std::string_view image_channel,
    std::function<void(cv::Mat, monotonic_clock::time_point,
                       std::vector<cv::Vec4i>,
                       std::vector<std::vector<cv::Point2f>>, bool,
                       std::vector<Eigen::Vector3d>,
                       std::vector<Eigen::Vector3d>)> &&handle_charuco_fn)
    : event_loop_(event_loop),
      target_type_(target_type),
      image_channel_(image_channel),
      calibration_(CHECK_NOTNULL(calibration)),
      handle_charuco_(std::move(handle_charuco_fn)) {
  SetupTargetData();

  LOG(INFO) << "Camera matrix " << calibration_.CameraIntrinsics();
  LOG(INFO) << "Distortion Coefficients " << calibration_.CameraDistCoeffs();

  LOG(INFO) << "Connecting to channel " << image_channel_;
}

void CharucoExtractor::HandleImage(cv::Mat rgb_image,
                                   const monotonic_clock::time_point eof) {
  const double age_double =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          event_loop_->monotonic_now() - eof)
          .count();

  // Have found this useful if there is blurry / noisy images
  if (FLAGS_gray_threshold > 0) {
    cv::Mat gray;
    cv::cvtColor(rgb_image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat thresh;
    cv::threshold(gray, thresh, FLAGS_gray_threshold, 255, cv::THRESH_BINARY);
    cv::cvtColor(thresh, rgb_image, cv::COLOR_GRAY2RGB);
  }

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
            calibration_.CameraIntrinsics(), calibration_.CameraDistCoeffs());

        if (charuco_ids.size() >= FLAGS_min_charucos) {
          cv::aruco::drawDetectedCornersCharuco(
              rgb_image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));

          cv::Vec3d rvec, tvec;
          valid = cv::aruco::estimatePoseCharucoBoard(
              charuco_corners_with_calibration, charuco_ids_with_calibration,
              board_, calibration_.CameraIntrinsics(),
              calibration_.CameraDistCoeffs(), rvec, tvec);

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
      cv::aruco::estimatePoseSingleMarkers(
          marker_corners, square_length_, calibration_.CameraIntrinsics(),
          calibration_.CameraDistCoeffs(), rvecs, tvecs);
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
        cv::aruco::estimatePoseSingleMarkers(
            diamond_corners, square_length_, calibration_.CameraIntrinsics(),
            calibration_.CameraDistCoeffs(), rvecs, tvecs);
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

flatbuffers::Offset<foxglove::ImageAnnotations> BuildAnnotations(
    const aos::monotonic_clock::time_point monotonic_now,
    const std::vector<std::vector<cv::Point2f>> &corners, double thickness,
    flatbuffers::FlatBufferBuilder *fbb) {
  std::vector<flatbuffers::Offset<foxglove::PointsAnnotation>> rectangles;
  const struct timespec now_t = aos::time::to_timespec(monotonic_now);
  foxglove::Time time{static_cast<uint32_t>(now_t.tv_sec),
                      static_cast<uint32_t>(now_t.tv_nsec)};
  // Draw the points in pink
  const flatbuffers::Offset<foxglove::Color> color_offset =
      foxglove::CreateColor(*fbb, 1.0, 0.75, 0.8, 1.0);
  for (const std::vector<cv::Point2f> &rectangle : corners) {
    std::vector<flatbuffers::Offset<foxglove::Point2>> points_offsets;
    for (const cv::Point2f &point : rectangle) {
      points_offsets.push_back(foxglove::CreatePoint2(*fbb, point.x, point.y));
    }
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<foxglove::Point2>>>
        points_offset = fbb->CreateVector(points_offsets);
    std::vector<flatbuffers::Offset<foxglove::Color>> color_offsets(
        points_offsets.size(), color_offset);
    auto colors_offset = fbb->CreateVector(color_offsets);
    foxglove::PointsAnnotation::Builder points_builder(*fbb);
    points_builder.add_timestamp(&time);
    points_builder.add_type(foxglove::PointsAnnotationType::POINTS);
    points_builder.add_points(points_offset);
    points_builder.add_outline_color(color_offset);
    points_builder.add_outline_colors(colors_offset);
    points_builder.add_thickness(thickness);
    rectangles.push_back(points_builder.Finish());
  }

  const auto rectangles_offset = fbb->CreateVector(rectangles);
  foxglove::ImageAnnotations::Builder annotation_builder(*fbb);
  annotation_builder.add_points(rectangles_offset);
  return annotation_builder.Finish();
}

TargetType TargetTypeFromString(std::string_view str) {
  if (str == "aruco") {
    return TargetType::kAruco;
  } else if (str == "charuco") {
    return TargetType::kCharuco;
  } else if (str == "charuco_diamond") {
    return TargetType::kCharucoDiamond;
  } else {
    LOG(FATAL) << "Unknown target type: " << str
               << ", expected: aruco|charuco|charuco_diamond";
  }
}

std::ostream &operator<<(std::ostream &os, TargetType target_type) {
  switch (target_type) {
    case TargetType::kAruco:
      os << "aruco";
      break;
    case TargetType::kCharuco:
      os << "charuco";
      break;
    case TargetType::kCharucoDiamond:
      os << "charuco_diamond";
      break;
  }
  return os;
}

}  // namespace vision
}  // namespace frc971
