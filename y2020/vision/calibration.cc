#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "absl/strings/str_format.h"
#include "y2020/vision/charuco_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"

DEFINE_string(config, "config.json", "Path to the config file to use.");
DEFINE_string(pi, "pi-7971-1", "Pi name to calibrate.");
DEFINE_string(calibration_folder, "y2020/vision/tools/python_code/calib_files",
              "Folder to place calibration files.");
DEFINE_bool(display_undistorted, false,
            "If true, display the undistorted image.");

namespace frc971 {
namespace vision {

class Calibration {
 public:
  Calibration(aos::ShmEventLoop *event_loop, std::string_view pi)
      : event_loop_(event_loop),
        pi_(pi),
        pi_number_(aos::network::ParsePiNumber(pi)),
        charuco_extractor_(
            event_loop, pi,
            [this](cv::Mat rgb_image, const double age_double,
                   std::vector<int> charuco_ids,
                   std::vector<cv::Point2f> charuco_corners, bool valid,
                   Eigen::Vector3d rvec_eigen, Eigen::Vector3d tvec_eigen) {
              HandleCharuco(rgb_image, age_double, charuco_ids, charuco_corners,
                            valid, rvec_eigen, tvec_eigen);
            }) {
    CHECK(pi_number_) << ": Invalid pi number " << pi
                      << ", failed to parse pi number";
  }

  void HandleCharuco(cv::Mat rgb_image, const double /*age_double*/,
                     std::vector<int> charuco_ids,
                     std::vector<cv::Point2f> charuco_corners, bool valid,
                     Eigen::Vector3d /*rvec_eigen*/,
                     Eigen::Vector3d /*tvec_eigen*/) {
    cv::imshow("Display", rgb_image);

    if (FLAGS_display_undistorted) {
      const cv::Size image_size(rgb_image.cols, rgb_image.rows);
      cv::Mat undistorted_rgb_image(image_size, CV_8UC3);
      cv::undistort(rgb_image, undistorted_rgb_image,
                    charuco_extractor_.camera_matrix(),
                    charuco_extractor_.dist_coeffs());

      cv::imshow("Display undist", undistorted_rgb_image);
    }

    int keystroke = cv::waitKey(1);
    if ((keystroke & 0xFF) == static_cast<int>('c')) {
      if (valid) {
        all_charuco_ids_.emplace_back(std::move(charuco_ids));
        all_charuco_corners_.emplace_back(std::move(charuco_corners));
        LOG(INFO) << "Image " << all_charuco_corners_.size();
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
          absl::StrFormat("/calibration_pi-%d-%d_%s.json", team_number.value(),
                          pi_number_.value(), time_ss.str());

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
  aos::ShmEventLoop *event_loop_;
  std::string pi_;
  const std::optional<uint16_t> pi_number_;

  std::vector<std::vector<int>> all_charuco_ids_;
  std::vector<std::vector<cv::Point2f>> all_charuco_corners_;

  CharucoExtractor charuco_extractor_;
};

namespace {

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  Calibration extractor(&event_loop, FLAGS_pi);

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
