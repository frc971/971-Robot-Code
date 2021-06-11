// These need to come before opencv, or they don't compile. Presumably opencv
// #defines something annoying.
// clang-format off
#include "Eigen/Dense"
#include "Eigen/Geometry"
// clang-format on

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "absl/strings/str_format.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "y2020/vision/sift/sift_generated.h"
#include "y2020/vision/sift/sift_training_generated.h"
#include "y2020/vision/tools/python_code/sift_training_data.h"
#include "y2020/vision/vision_generated.h"

DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_string(pi, "pi-7971-1", "Pi name to calibrate.");
DEFINE_string(calibration_folder, "y2020/vision/tools/python_code/calib_files",
              "Folder to place calibration files.");
DEFINE_bool(large_board, true, "If true, use the large calibration board.");
DEFINE_bool(display_undistorted, false,
            "If true, display the undistorted image.");
DEFINE_string(board_template_path, "",
              "If specified, write an image to the specified path for the "
              "charuco board pattern.");
DEFINE_uint32(min_targets, 10,
              "The mininum number of targets required to match.");

namespace frc971 {
namespace vision {

class CameraCalibration {
 public:
  CameraCalibration(const absl::Span<const uint8_t> training_data_bfbs) {
    const aos::FlatbufferSpan<sift::TrainingData> training_data(
        training_data_bfbs);
    CHECK(training_data.Verify());
    camera_calibration_ = FindCameraCalibration(&training_data.message());
  }

  cv::Mat CameraIntrinsics() const {
    const cv::Mat result(3, 3, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             camera_calibration_->intrinsics()->data())));
    CHECK_EQ(result.total(), camera_calibration_->intrinsics()->size());
    return result;
  }

  cv::Mat CameraDistCoeffs() const {
    const cv::Mat result(5, 1, CV_32F,
                         const_cast<void *>(static_cast<const void *>(
                             camera_calibration_->dist_coeffs()->data())));
    CHECK_EQ(result.total(), camera_calibration_->dist_coeffs()->size());
    return result;
  }

 private:
  const sift::CameraCalibration *FindCameraCalibration(
      const sift::TrainingData *const training_data) const {
    std::optional<uint16_t> pi_number = aos::network::ParsePiNumber(FLAGS_pi);
    std::optional<uint16_t> team_number =
        aos::network::team_number_internal::ParsePiTeamNumber(FLAGS_pi);
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

  const sift::CameraCalibration *camera_calibration_;
};

namespace {

void ViewerMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  const CameraCalibration calibration(SiftTrainingData());

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(FLAGS_large_board
                                             ? cv::aruco::DICT_5X5_100
                                             : cv::aruco::DICT_6X6_250);
  cv::Ptr<cv::aruco::CharucoBoard> board =
      FLAGS_large_board
          ? cv::aruco::CharucoBoard::create(12, 9, 0.06, 0.045, dictionary)
          : cv::aruco::CharucoBoard::create(7, 5, 0.04, 0.025, dictionary);

  if (!FLAGS_board_template_path.empty()) {
    cv::Mat board_image;
    board->draw(cv::Size(600, 500), board_image, 10, 1);
    cv::imwrite(FLAGS_board_template_path, board_image);
  }

  std::vector<std::vector<int>> all_charuco_ids;
  std::vector<std::vector<cv::Point2f>> all_charuco_corners;

  const cv::Mat camera_matrix = calibration.CameraIntrinsics();
  Eigen::Matrix3d eigen_camera_matrix;
  cv::cv2eigen(camera_matrix, eigen_camera_matrix);

  const cv::Mat dist_coeffs = calibration.CameraDistCoeffs();
  LOG(INFO) << "Camera matrix " << camera_matrix;
  LOG(INFO) << "Distortion Coefficients " << dist_coeffs;

  std::optional<uint16_t> pi_number = aos::network::ParsePiNumber(FLAGS_pi);
  CHECK(pi_number) << ": Invalid pi number " << FLAGS_pi
                   << ", failed to parse pi number";

  const std::string channel =
      absl::StrCat("/pi", std::to_string(pi_number.value()), "/camera");
  LOG(INFO) << "Connecting to channel " << channel;

  event_loop.MakeWatcher(channel, [&event_loop, &board, &all_charuco_ids,
                                   &all_charuco_corners, camera_matrix,
                                   dist_coeffs, eigen_camera_matrix](
                                      const CameraImage &image) {
    const aos::monotonic_clock::duration age =
        event_loop.monotonic_now() - event_loop.context().monotonic_event_time;
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
    cv::cvtColor(image_color_mat, rgb_image, CV_YUV2BGR_YUYV);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(rgb_image, board->dictionary, marker_corners,
                             marker_ids);

    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    // If at least one marker detected
    if (marker_ids.size() >= FLAGS_min_targets) {
      // Run everything twice, once with the calibration, and once without.
      // This lets us both calibrate, and also print out the pose real time
      // with the previous calibration.
      cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids,
                                           rgb_image, board, charuco_corners,
                                           charuco_ids);

      std::vector<cv::Point2f> charuco_corners_with_calibration;
      std::vector<int> charuco_ids_with_calibration;

      cv::aruco::interpolateCornersCharuco(
          marker_corners, marker_ids, rgb_image, board,
          charuco_corners_with_calibration, charuco_ids_with_calibration,
          camera_matrix, dist_coeffs);

      cv::aruco::drawDetectedMarkers(rgb_image, marker_corners, marker_ids);

      if (charuco_ids.size() >= FLAGS_min_targets) {
        cv::aruco::drawDetectedCornersCharuco(
            rgb_image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));

        cv::Vec3d rvec, tvec;
        bool valid = cv::aruco::estimatePoseCharucoBoard(
            charuco_corners_with_calibration, charuco_ids_with_calibration,
            board, camera_matrix, dist_coeffs, rvec, tvec);

        // if charuco pose is valid
        if (valid) {
          LOG(INFO) << std::fixed << std::setprecision(6)
                    << "Age: " << age_double << ", Pose is R:" << rvec
                    << " T:" << tvec;
          cv::aruco::drawAxis(rgb_image, camera_matrix, dist_coeffs, rvec, tvec,
                              0.1);
        } else {
          LOG(INFO) << "Age: " << age_double << ", invalid pose";
        }
      } else {
        LOG(INFO) << "Age: " << age_double << ", no charuco IDs.";
      }
    } else {
      LOG(INFO) << "Age: " << age_double << ", no marker IDs";
    }

    cv::imshow("Display", rgb_image);

    if (FLAGS_display_undistorted) {
      cv::Mat undistorted_rgb_image(image_size, CV_8UC3);
      cv::undistort(rgb_image, undistorted_rgb_image, camera_matrix,
                    dist_coeffs);

      cv::imshow("Display undist", undistorted_rgb_image);
    }

    int keystroke = cv::waitKey(1);
    if ((keystroke & 0xFF) == static_cast<int>('c')) {
      if (charuco_ids.size() >= FLAGS_min_targets) {
        all_charuco_ids.emplace_back(std::move(charuco_ids));
        all_charuco_corners.emplace_back(std::move(charuco_corners));
        LOG(INFO) << "Image " << all_charuco_corners.size();
      }

      if (all_charuco_ids.size() >= 50) {
        LOG(INFO) << "Got enough images to calibrate";
        event_loop.Exit();
      }
    } else if ((keystroke & 0xFF) == static_cast<int>('q')) {
      event_loop.Exit();
    }
  });

  event_loop.Run();

  if (all_charuco_ids.size() >= 50) {
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;

    // Set calibration flags (same than in calibrateCamera() function)
    int calibration_flags = 0;
    cv::Size img_size(640, 480);
    const double reprojection_error = cv::aruco::calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, img_size, cameraMatrix,
        distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics,
        stdDeviationsExtrinsics, perViewErrors, calibration_flags);
    CHECK_LE(reprojection_error, 1.0) << ": Reproduction error is bad.";
    LOG(INFO) << "Reprojection Error is " << reprojection_error;

    flatbuffers::FlatBufferBuilder fbb;
    flatbuffers::Offset<flatbuffers::String> name_offset =
        fbb.CreateString(absl::StrFormat("pi%d", pi_number.value()));
    flatbuffers::Offset<flatbuffers::Vector<float>> intrinsics_offset =
        fbb.CreateVector<float>(9u, [&cameraMatrix](size_t i) {
          return static_cast<float>(
              reinterpret_cast<double *>(cameraMatrix.data)[i]);
        });

    flatbuffers::Offset<flatbuffers::Vector<float>>
        distortion_coefficients_offset =
            fbb.CreateVector<float>(5u, [&distCoeffs](size_t i) {
              return static_cast<float>(
                  reinterpret_cast<double *>(distCoeffs.data)[i]);
            });

    sift::CameraCalibration::Builder camera_calibration_builder(fbb);
    std::optional<uint16_t> team_number =
        aos::network::team_number_internal::ParsePiTeamNumber(FLAGS_pi);
    CHECK(team_number) << ": Invalid pi hostname " << FLAGS_pi
                       << ", failed to parse team number";

    const aos::realtime_clock::time_point realtime_now =
        aos::realtime_clock::now();
    camera_calibration_builder.add_node_name(name_offset);
    camera_calibration_builder.add_team_number(team_number.value());
    camera_calibration_builder.add_calibration_timestamp(
        realtime_now.time_since_epoch().count());
    camera_calibration_builder.add_intrinsics(intrinsics_offset);
    camera_calibration_builder.add_dist_coeffs(distortion_coefficients_offset);
    camera_calibration_builder.add_dist_coeffs(distortion_coefficients_offset);
    fbb.Finish(camera_calibration_builder.Finish());

    aos::FlatbufferDetachedBuffer<sift::CameraCalibration> camera_calibration(
        fbb.Release());
    std::stringstream time_ss;
    time_ss << realtime_now;

    const std::string calibration_filename =
        FLAGS_calibration_folder +
        absl::StrFormat("/calibration_pi-%d-%d_%s.json", team_number.value(),
                        pi_number.value(), time_ss.str());

    LOG(INFO) << calibration_filename << " -> "
              << aos::FlatbufferToJson(camera_calibration,
                                       {.multi_line = true});

    aos::util::WriteStringToFileOrDie(
        calibration_filename,
        aos::FlatbufferToJson(camera_calibration, {.multi_line = true}));
  } else {
    LOG(INFO) << "Skipping calibration due to not enough images.";
  }
}

}  // namespace
}  // namespace vision
}  // namespace frc971

// Quick and lightweight grayscale viewer for images
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::ViewerMain();
}
