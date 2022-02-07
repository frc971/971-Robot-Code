#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <regex>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/strings/str_format.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "y2020/vision/charuco_lib.h"

DEFINE_string(calibration_folder, ".", "Folder to place calibration files.");
DEFINE_string(camera_id, "", "Camera ID in format YY-NN-- year and number.");
DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_bool(display_undistorted, false,
            "If true, display the undistorted image.");
DEFINE_string(pi, "", "Pi name to calibrate.");

namespace frc971 {
namespace vision {

class Calibration {
 public:
  Calibration(aos::ShmEventLoop *event_loop, std::string_view pi,
              std::string_view camera_id)
      : event_loop_(event_loop),
        pi_(pi),
        pi_number_(aos::network::ParsePiNumber(pi)),
        camera_id_(camera_id),
        H_camera_board_(Eigen::Affine3d()),
        prev_H_camera_board_(Eigen::Affine3d()),
        charuco_extractor_(
            event_loop, pi,
            [this](cv::Mat rgb_image,
                   const aos::monotonic_clock::time_point eof,
                   std::vector<int> charuco_ids,
                   std::vector<cv::Point2f> charuco_corners, bool valid,
                   Eigen::Vector3d rvec_eigen, Eigen::Vector3d tvec_eigen) {
              HandleCharuco(rgb_image, eof, charuco_ids, charuco_corners, valid,
                            rvec_eigen, tvec_eigen);
            }) {
    CHECK(pi_number_) << ": Invalid pi number " << pi
                      << ", failed to parse pi number";
    std::regex re{"^[0-9][0-9]-[0-9][0-9]"};
    CHECK(std::regex_match(camera_id_, re))
        << ": Invalid camera_id '" << camera_id_
        << "', should be of form YY-NN";
  }

  void HandleCharuco(cv::Mat rgb_image,
                     const aos::monotonic_clock::time_point /*eof*/,
                     std::vector<int> charuco_ids,
                     std::vector<cv::Point2f> charuco_corners, bool valid,
                     Eigen::Vector3d rvec_eigen, Eigen::Vector3d tvec_eigen) {
    // Reduce resolution displayed on remote viewer to prevent lag
    cv::resize(rgb_image, rgb_image,
               cv::Size(rgb_image.cols / 2, rgb_image.rows / 2));
    cv::imshow("Display", rgb_image);

    if (FLAGS_display_undistorted) {
      const cv::Size image_size(rgb_image.cols, rgb_image.rows);
      cv::Mat undistorted_rgb_image(image_size, CV_8UC3);
      cv::undistort(rgb_image, undistorted_rgb_image,
                    charuco_extractor_.camera_matrix(),
                    charuco_extractor_.dist_coeffs());

      cv::imshow("Display undist", undistorted_rgb_image);
    }

    // Calibration calculates rotation and translation delta from last image
    // captured to automatically capture next image

    Eigen::Affine3d H_board_camera =
        Eigen::Translation3d(tvec_eigen) *
        Eigen::AngleAxisd(rvec_eigen.norm(), rvec_eigen / rvec_eigen.norm());
    H_camera_board_ = H_board_camera.inverse();
    Eigen::Affine3d H_delta = H_board_camera * prev_H_camera_board_;

    Eigen::AngleAxisd delta_r = Eigen::AngleAxisd(H_delta.rotation());

    Eigen::Vector3d delta_t = H_delta.translation();

    double r_norm = std::abs(delta_r.angle());
    double t_norm = delta_t.norm();

    int keystroke = cv::waitKey(1);
    if (r_norm > kDeltaRThreshold || t_norm > kDeltaTThreshold) {
      if (valid) {
        prev_H_camera_board_ = H_camera_board_;

        all_charuco_ids_.emplace_back(std::move(charuco_ids));
        all_charuco_corners_.emplace_back(std::move(charuco_corners));

        if (r_norm > kDeltaRThreshold) {
          LOG(INFO) << "Triggered by rotation delta = " << r_norm << " > "
                    << kDeltaRThreshold;
        }
        if (t_norm > kDeltaTThreshold) {
          LOG(INFO) << "Trigerred by translation delta = " << t_norm << " > "
                    << kDeltaTThreshold;
        }

        LOG(INFO) << "Image count " << all_charuco_corners_.size();
      }

      if (all_charuco_ids_.size() >= 50) {
        LOG(INFO) << "Got enough images to calibrate";
        event_loop_->Exit();
      }
    } else if ((keystroke & 0xFF) == static_cast<int>('q')) {
      event_loop_->Exit();
    }
  }

  void MaybeCalibrate() {
    if (all_charuco_ids_.size() >= 50) {
      LOG(INFO) << "Beginning calibration on " << all_charuco_ids_.size()
                << " images";
      cv::Mat cameraMatrix, distCoeffs;
      std::vector<cv::Mat> rvecs, tvecs;
      cv::Mat stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;

      // Set calibration flags (same as in calibrateCamera() function)
      int calibration_flags = 0;
      cv::Size img_size(640, 480);
      const double reprojection_error = cv::aruco::calibrateCameraCharuco(
          all_charuco_corners_, all_charuco_ids_, charuco_extractor_.board(),
          img_size, cameraMatrix, distCoeffs, rvecs, tvecs,
          stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors,
          calibration_flags);
      CHECK_LE(reprojection_error, 1.0) << ": Reproduction error is bad.";
      LOG(INFO) << "Reprojection Error is " << reprojection_error;

      flatbuffers::FlatBufferBuilder fbb;
      flatbuffers::Offset<flatbuffers::String> name_offset =
          fbb.CreateString(absl::StrFormat("pi%d", pi_number_.value()));
      flatbuffers::Offset<flatbuffers::String> camera_id_offset =
          fbb.CreateString(camera_id_);
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
          aos::network::team_number_internal::ParsePiTeamNumber(pi_);
      CHECK(team_number) << ": Invalid pi hostname " << pi_
                         << ", failed to parse team number";

      const aos::realtime_clock::time_point realtime_now =
          aos::realtime_clock::now();
      camera_calibration_builder.add_node_name(name_offset);
      camera_calibration_builder.add_team_number(team_number.value());
      camera_calibration_builder.add_camera_id(camera_id_offset);
      camera_calibration_builder.add_calibration_timestamp(
          realtime_now.time_since_epoch().count());
      camera_calibration_builder.add_intrinsics(intrinsics_offset);
      camera_calibration_builder.add_dist_coeffs(
          distortion_coefficients_offset);
      fbb.Finish(camera_calibration_builder.Finish());

      aos::FlatbufferDetachedBuffer<sift::CameraCalibration> camera_calibration(
          fbb.Release());
      std::stringstream time_ss;
      time_ss << realtime_now;

      const std::string calibration_filename =
          FLAGS_calibration_folder +
          absl::StrFormat("/calibration_pi-%d-%d_cam-%s_%s.json",
                          team_number.value(), pi_number_.value(), camera_id_,
                          time_ss.str());

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

 private:
  static constexpr double kDeltaRThreshold = M_PI / 6.0;
  static constexpr double kDeltaTThreshold = 0.3;

  aos::ShmEventLoop *event_loop_;
  std::string pi_;
  const std::optional<uint16_t> pi_number_;
  const std::string camera_id_;

  std::vector<std::vector<int>> all_charuco_ids_;
  std::vector<std::vector<cv::Point2f>> all_charuco_corners_;

  Eigen::Affine3d H_camera_board_;
  Eigen::Affine3d prev_H_camera_board_;

  CharucoExtractor charuco_extractor_;
};

namespace {

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  std::string hostname = FLAGS_pi;
  if (hostname == "") {
    hostname = aos::network::GetHostname();
    LOG(INFO) << "Using pi name from hostname as " << hostname;
  }
  Calibration extractor(&event_loop, hostname, FLAGS_camera_id);

  event_loop.Run();

  extractor.MaybeCalibrate();
}

}  // namespace
}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::Main();
}
