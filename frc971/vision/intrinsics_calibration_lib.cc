#include "frc971/vision/intrinsics_calibration_lib.h"

namespace frc971 {
namespace vision {

IntrinsicsCalibration::IntrinsicsCalibration(
    aos::EventLoop *event_loop, std::string_view pi, std::string_view camera_id,
    std::string_view base_intrinsics_file, bool display_undistorted,
    std::string_view calibration_folder, aos::ExitHandle *exit_handle)
    : pi_(pi),
      pi_number_(aos::network::ParsePiNumber(pi)),
      camera_id_(camera_id),
      prev_H_camera_board_(Eigen::Affine3d()),
      prev_image_H_camera_board_(Eigen::Affine3d()),
      base_intrinsics_(
          aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
              base_intrinsics_file)),
      charuco_extractor_(
          event_loop, &base_intrinsics_.message(), TargetType::kCharuco,
          "/camera",
          [this](cv::Mat rgb_image, const aos::monotonic_clock::time_point eof,
                 std::vector<cv::Vec4i> charuco_ids,
                 std::vector<std::vector<cv::Point2f>> charuco_corners,
                 bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                 std::vector<Eigen::Vector3d> tvecs_eigen) {
            HandleCharuco(rgb_image, eof, charuco_ids, charuco_corners, valid,
                          rvecs_eigen, tvecs_eigen);
          }),
      image_callback_(
          event_loop,
          absl::StrCat("/pi",
                       std::to_string(aos::network::ParsePiNumber(pi).value()),
                       "/camera"),
          [this](cv::Mat rgb_image,
                 const aos::monotonic_clock::time_point eof) {
            charuco_extractor_.HandleImage(rgb_image, eof);
          },
          std::chrono::milliseconds(5)),
      display_undistorted_(display_undistorted),
      calibration_folder_(calibration_folder),
      exit_handle_(exit_handle) {
  CHECK(pi_number_) << ": Invalid pi number " << pi
                    << ", failed to parse pi number";
  std::regex re{"^[0-9][0-9]-[0-9][0-9]"};
  CHECK(std::regex_match(camera_id_, re))
      << ": Invalid camera_id '" << camera_id_ << "', should be of form YY-NN";
}

void IntrinsicsCalibration::HandleCharuco(
    cv::Mat rgb_image, const aos::monotonic_clock::time_point /*eof*/,
    std::vector<cv::Vec4i> charuco_ids,
    std::vector<std::vector<cv::Point2f>> charuco_corners, bool valid,
    std::vector<Eigen::Vector3d> rvecs_eigen,
    std::vector<Eigen::Vector3d> tvecs_eigen) {
  // Reduce resolution displayed on remote viewer to prevent lag
  cv::resize(rgb_image, rgb_image,
             cv::Size(rgb_image.cols / 2, rgb_image.rows / 2));
  cv::imshow("Display", rgb_image);

  if (display_undistorted_) {
    const cv::Size image_size(rgb_image.cols, rgb_image.rows);
    cv::Mat undistorted_rgb_image(image_size, CV_8UC3);
    cv::undistort(rgb_image, undistorted_rgb_image,
                  charuco_extractor_.camera_matrix(),
                  charuco_extractor_.dist_coeffs());

    cv::imshow("Display undist", undistorted_rgb_image);
  }

  int keystroke = cv::waitKey(1);

  // If we haven't got a valid pose estimate, don't use these points
  if (!valid) {
    return;
  }
  CHECK(tvecs_eigen.size() == 1)
      << "Charuco board should only return one translational pose";
  CHECK(rvecs_eigen.size() == 1)
      << "Charuco board should only return one rotational pose";
  // Calibration calculates rotation and translation delta from last image
  // stored to automatically capture next image

  Eigen::Affine3d H_board_camera =
      Eigen::Translation3d(tvecs_eigen[0]) *
      Eigen::AngleAxisd(rvecs_eigen[0].norm(),
                        rvecs_eigen[0] / rvecs_eigen[0].norm());
  Eigen::Affine3d H_camera_board_ = H_board_camera.inverse();
  Eigen::Affine3d H_delta = H_board_camera * prev_H_camera_board_;

  Eigen::AngleAxisd delta_r = Eigen::AngleAxisd(H_delta.rotation());

  Eigen::Vector3d delta_t = H_delta.translation();

  double r_norm = std::abs(delta_r.angle());
  double t_norm = delta_t.norm();

  bool store_image = false;
  double percent_motion =
      std::max<double>(r_norm / kDeltaRThreshold, t_norm / kDeltaTThreshold);
  LOG(INFO) << "Captured: " << all_charuco_ids_.size() << " points; Moved "
            << percent_motion << "% of what's needed";
  // Verify that camera has moved enough from last stored image
  if (r_norm > kDeltaRThreshold || t_norm > kDeltaTThreshold) {
    // frame_ refers to deltas between current and last captured image
    Eigen::Affine3d frame_H_delta = H_board_camera * prev_image_H_camera_board_;

    Eigen::AngleAxisd frame_delta_r =
        Eigen::AngleAxisd(frame_H_delta.rotation());

    Eigen::Vector3d frame_delta_t = frame_H_delta.translation();

    double frame_r_norm = std::abs(frame_delta_r.angle());
    double frame_t_norm = frame_delta_t.norm();

    // Make sure camera has stopped moving before storing image
    store_image =
        frame_r_norm < kFrameDeltaRLimit && frame_t_norm < kFrameDeltaTLimit;
    double percent_stop = std::max<double>(frame_r_norm / kFrameDeltaRLimit,
                                           frame_t_norm / kFrameDeltaTLimit);
    LOG(INFO) << "Captured: " << all_charuco_ids_.size()
              << "points; Moved enough (" << percent_motion
              << "%); Need to stop (last motion was " << percent_stop
              << "% of limit; needs to be < 1 to capture)";
  }
  prev_image_H_camera_board_ = H_camera_board_;

  if (store_image) {
    if (valid) {
      prev_H_camera_board_ = H_camera_board_;

      // Unpack the Charuco ids from Vec4i
      std::vector<int> charuco_ids_int;
      for (cv::Vec4i charuco_id : charuco_ids) {
        charuco_ids_int.emplace_back(charuco_id[0]);
      }
      all_charuco_ids_.emplace_back(std::move(charuco_ids_int));
      all_charuco_corners_.emplace_back(std::move(charuco_corners[0]));

      if (r_norm > kDeltaRThreshold) {
        LOG(INFO) << "Triggered by rotation delta = " << r_norm << " > "
                  << kDeltaRThreshold;
      }
      if (t_norm > kDeltaTThreshold) {
        LOG(INFO) << "Triggered by translation delta = " << t_norm << " > "
                  << kDeltaTThreshold;
      }
    }

  } else if ((keystroke & 0xFF) == static_cast<int>('q')) {
    exit_handle_->Exit();
  }
}

aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
IntrinsicsCalibration::BuildCalibration(
    cv::Mat camera_matrix, cv::Mat dist_coeffs,
    aos::realtime_clock::time_point realtime_now, int pi_number,
    std::string_view camera_id, uint16_t team_number) {
  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb.CreateString(absl::StrFormat("pi%d", pi_number));
  flatbuffers::Offset<flatbuffers::String> camera_id_offset =
      fbb.CreateString(camera_id);
  flatbuffers::Offset<flatbuffers::Vector<float>> intrinsics_offset =
      fbb.CreateVector<float>(9u, [&camera_matrix](size_t i) {
        return static_cast<float>(
            reinterpret_cast<double *>(camera_matrix.data)[i]);
      });

  flatbuffers::Offset<flatbuffers::Vector<float>>
      distortion_coefficients_offset =
          fbb.CreateVector<float>(5u, [&dist_coeffs](size_t i) {
            return static_cast<float>(
                reinterpret_cast<double *>(dist_coeffs.data)[i]);
          });

  calibration::CameraCalibration::Builder camera_calibration_builder(fbb);

  camera_calibration_builder.add_node_name(name_offset);
  camera_calibration_builder.add_team_number(team_number);
  camera_calibration_builder.add_camera_id(camera_id_offset);
  camera_calibration_builder.add_calibration_timestamp(
      realtime_now.time_since_epoch().count());
  camera_calibration_builder.add_intrinsics(intrinsics_offset);
  camera_calibration_builder.add_dist_coeffs(distortion_coefficients_offset);
  fbb.Finish(camera_calibration_builder.Finish());

  return fbb.Release();
}

void IntrinsicsCalibration::MaybeCalibrate() {
  // TODO: This number should depend on coarse vs. fine pattern
  // Maybe just on total # of ids found, not just images
  if (all_charuco_ids_.size() >= 50) {
    LOG(INFO) << "Beginning calibration on " << all_charuco_ids_.size()
              << " images";
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat std_deviations_intrinsics, std_deviations_extrinsics,
        per_view_errors;

    // Set calibration flags (same as in calibrateCamera() function)
    int calibration_flags = 0;
    cv::Size img_size(640, 480);
    const double reprojection_error = cv::aruco::calibrateCameraCharuco(
        all_charuco_corners_, all_charuco_ids_, charuco_extractor_.board(),
        img_size, camera_matrix, dist_coeffs, rvecs, tvecs,
        std_deviations_intrinsics, std_deviations_extrinsics, per_view_errors,
        calibration_flags);
    CHECK_LE(reprojection_error, 1.0)
        << ": Reproduction error is bad-- greater than 1 pixel.";
    LOG(INFO) << "Reprojection Error is " << reprojection_error;

    const aos::realtime_clock::time_point realtime_now =
        aos::realtime_clock::now();
    std::optional<uint16_t> team_number =
        aos::network::team_number_internal::ParsePiTeamNumber(pi_);
    CHECK(team_number) << ": Invalid pi hostname " << pi_
                       << ", failed to parse team number";
    aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
        camera_calibration = BuildCalibration(camera_matrix, dist_coeffs,
                                              realtime_now, pi_number_.value(),
                                              camera_id_, team_number.value());
    std::stringstream time_ss;
    time_ss << realtime_now;

    const std::string calibration_filename =
        calibration_folder_ +
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
}  // namespace vision
}  // namespace frc971
