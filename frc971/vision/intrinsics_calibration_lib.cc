#include "frc971/vision/intrinsics_calibration_lib.h"

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"

ABSL_FLAG(std::string, image_save_path, "",
          "If specified, save out images that are captured for calibraiton");

// NOTE: This will flip any annotations / text that has already been drawn on
// the image (e.g., in charuco_lib.cc)
ABSL_FLAG(bool, flip_image, true,
          "If true, flip the display image so visualization looks correct "
          "during calibration");

ABSL_DECLARE_FLAG(bool, visualize);

namespace frc971::vision {

// Found that under 50 ms would fail image too often on intrinsics with
// visualize on
constexpr aos::monotonic_clock::duration kMaxImageAge =
    aos::monotonic_clock::duration(std::chrono::milliseconds(50));

IntrinsicsCalibration::IntrinsicsCalibration(
    aos::EventLoop *event_loop, std::string_view hostname,
    std::string_view camera_channel, std::string_view camera_id,
    std::string_view base_intrinsics_file, bool display_undistorted,
    std::string_view calibration_folder, aos::ExitHandle *exit_handle)
    : hostname_(hostname),
      cpu_type_(aos::network::ParsePiOrOrin(hostname_)),
      cpu_number_(aos::network::ParsePiOrOrinNumber(hostname_)),
      camera_channel_(camera_channel),
      camera_id_(camera_id),
      prev_H_board_camera_(Eigen::Affine3d()),
      last_frame_H_board_camera_(Eigen::Affine3d()),
      base_intrinsics_(
          aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
              base_intrinsics_file)),
      charuco_extractor_(
          event_loop, &base_intrinsics_.message(), TargetType::kCharuco,
          camera_channel_,
          [this](cv::Mat rgb_image, const aos::monotonic_clock::time_point eof,
                 std::vector<cv::Vec4i> charuco_ids,
                 std::vector<std::vector<cv::Point2f>> charuco_corners,
                 bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                 std::vector<Eigen::Vector3d> tvecs_eigen) {
            HandleCharuco(rgb_image, eof, charuco_ids, charuco_corners, valid,
                          rvecs_eigen, tvecs_eigen);
          }),
      image_callback_(
          event_loop, camera_channel_,
          [this](cv::Mat rgb_image,
                 const aos::monotonic_clock::time_point eof) {
            if (exit_collection_) {
              return;
            }
            charuco_extractor_.HandleImage(rgb_image, eof);
          },
          kMaxImageAge),
      display_undistorted_(display_undistorted),
      calibration_folder_(calibration_folder),
      exit_handle_(exit_handle),
      exit_collection_(false) {
  if (!absl::GetFlag(FLAGS_visualize)) {
    // The only way to exit into the calibration routines is by hitting "q"
    // while visualization is running.  The event_loop doesn't pause enough
    // to handle ctrl-c exit requests
    LOG(INFO) << "Setting visualize to true, since currently the intrinsics "
                 "only works this way";
    absl::SetFlag(&FLAGS_visualize, true);
  }

  CHECK((absl::GetFlag(FLAGS_image_save_path) == "") ||
        !absl::GetFlag(FLAGS_draw_axes))
      << "Only save images if we're not drawing on them";

  LOG(INFO) << "Hostname is: " << hostname_ << " and camera channel is "
            << camera_channel_;

  CHECK(cpu_number_) << ": Invalid cpu number " << hostname_
                     << ", failed to parse cpu number";
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
  constexpr float kVisualizationScaleFactor = 2.0;
  static int image_save_count = 0;
  if (absl::GetFlag(FLAGS_image_save_path) != "") {
    std::string image_name =
        absl::StrFormat("/img_%06d.png", image_save_count++);
    std::string path = absl::GetFlag(FLAGS_image_save_path) + image_name;
    VLOG(2) << "Saving intrinsic calibration image to " << path;
    cv::imwrite(path, rgb_image);
  }

  // Store the image size, since it gets used in undistort and calibration
  image_size_ = rgb_image.size();
  if (absl::GetFlag(FLAGS_visualize)) {
    // Before resizing/flipping, display undistorted image, if asked to
    if (display_undistorted_) {
      cv::Mat undistorted_rgb_image(image_size_, CV_8UC3);
      cv::undistort(rgb_image, undistorted_rgb_image,
                    charuco_extractor_.camera_matrix(),
                    charuco_extractor_.dist_coeffs());

      cv::imshow("Display undist", undistorted_rgb_image);
    }

    // Reduce resolution displayed on remote viewer to prevent lag
    cv::resize(rgb_image, rgb_image,
               cv::Size(rgb_image.cols / kVisualizationScaleFactor,
                        rgb_image.rows / kVisualizationScaleFactor));
    cv::Mat display_image = rgb_image;
    if (absl::GetFlag(FLAGS_flip_image)) {
      cv::flip(rgb_image, display_image, 1);
    }
    cv::putText(
        display_image,
        "Captured " + std::to_string(all_charuco_ids_.size()) + " images",
        cv::Point(display_image.size().width / 2, 10), cv::FONT_HERSHEY_PLAIN,
        1.0, cv::Scalar(0, 255, 0));
    if (!valid) {
      cv::putText(display_image, "Invalid Estimated Pose",
                  cv::Point(display_image.size().width / 2, 30),
                  cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
    }
    cv::imshow("Display", display_image);
  }

  int keystroke = cv::waitKey(1);
  if ((keystroke & 0xFF) == static_cast<int>('q')) {
    LOG(INFO) << "Going to exit";
    exit_collection_ = true;
    exit_handle_->Exit();
  }
  // If we haven't got a valid pose estimate, don't use these points
  if (!valid) {
    LOG(INFO) << "Skipping because pose is not valid";
    return;
  }
  CHECK(tvecs_eigen.size() == 1)
      << "Charuco board should only return one translational pose";
  CHECK(rvecs_eigen.size() == 1)
      << "Charuco board should only return one rotational pose";

  // Calculate rotation and translation delta from last image stored to
  // determine when to capture next image
  Eigen::Affine3d H_camera_board =
      Eigen::Translation3d(tvecs_eigen[0]) *
      Eigen::AngleAxisd(rvecs_eigen[0].norm(),
                        rvecs_eigen[0] / rvecs_eigen[0].norm());
  Eigen::Affine3d H_delta_camera = H_camera_board * prev_H_board_camera_;

  Eigen::AngleAxisd delta_r = Eigen::AngleAxisd(H_delta_camera.rotation());
  Eigen::Vector3d delta_t = H_delta_camera.translation();

  double r_norm = std::abs(delta_r.angle());
  double t_norm = delta_t.norm();

  bool store_image = false;
  double percent_motion =
      std::max<double>(r_norm / kDeltaRThreshold, t_norm / kDeltaTThreshold);
  LOG(INFO) << "Captured: " << all_charuco_ids_.size() << " samples; \nMoved "
            << static_cast<int>(percent_motion * 100) << "% of what's needed";

  // Verify that camera has moved enough from last stored image
  if (r_norm > kDeltaRThreshold || t_norm > kDeltaTThreshold) {
    // frame_ refers to deltas between current and last captured image
    Eigen::Affine3d frame_H_delta_camera =
        H_camera_board * last_frame_H_board_camera_;

    Eigen::AngleAxisd frame_delta_r =
        Eigen::AngleAxisd(frame_H_delta_camera.rotation());
    Eigen::Vector3d frame_delta_t = frame_H_delta_camera.translation();

    double frame_r_norm = std::abs(frame_delta_r.angle());
    double frame_t_norm = frame_delta_t.norm();

    // Make sure camera has stopped moving before storing image
    store_image =
        frame_r_norm < kFrameDeltaRLimit && frame_t_norm < kFrameDeltaTLimit;
    double percent_stop = std::max<double>(frame_r_norm / kFrameDeltaRLimit,
                                           frame_t_norm / kFrameDeltaTLimit);
    LOG(INFO) << "\nCaptured: " << all_charuco_ids_.size()
              << " samples; \nMoved enough ("
              << static_cast<int>(percent_motion * 100)
              << "%); Last motion was " << static_cast<int>(percent_stop * 100)
              << "% of limit. "
              << (store_image ? "Capturing image" : "Still need to stop");
  }
  last_frame_H_board_camera_ = H_camera_board.inverse();

  if (all_charuco_ids_.empty() && valid) {
    // If we haven't captured yet, and it's a valid pose, go ahead and store
    // it, since we don't have a previous pose to reference
    LOG(INFO) << "Capturing first valid pose estimate";
    store_image = true;
  }

  if (store_image) {
    if (valid) {
      prev_H_board_camera_ = H_camera_board.inverse();

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
      if (absl::GetFlag(FLAGS_visualize)) {
        if (point_viz_image_.empty()) {
          point_viz_image_ = cv::Mat::zeros(rgb_image.size(), CV_8UC1);
        }
        // Draw a small circle for each corner
        for (uint i = 0; i < all_charuco_corners_.back().size(); i++) {
          cv::Point2f point = all_charuco_corners_.back()[i];
          point = point / kVisualizationScaleFactor;
          cv::circle(point_viz_image_, point, 3, cv::Scalar(255, 255, 255));
        }

        cv::Mat display_image;
        if (absl::GetFlag(FLAGS_flip_image)) {
          cv::flip(point_viz_image_, display_image, 1);
        } else {
          display_image = point_viz_image_;
        }
        cv::imshow("Captured Point Visualization", display_image);
        cv::waitKey(1);
      }
    }
  }
}

aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
IntrinsicsCalibration::BuildCalibration(
    cv::Mat camera_matrix, cv::Mat dist_coeffs,
    aos::realtime_clock::time_point realtime_now, std::string_view cpu_type,
    uint16_t cpu_number, std::string_view camera_channel,
    std::string_view camera_id, uint16_t team_number,
    double reprojection_error) {
  flatbuffers::FlatBufferBuilder fbb;
  // THIS IS A HACK FOR 2024, since we call Orin2 "Imu"
  std::string cpu_name = absl::StrFormat("%s%d", cpu_type, cpu_number);
  if (cpu_type == "orin" && cpu_number == 2) {
    LOG(INFO) << "Renaming orin2 to imu";
    cpu_name = "imu";
  }
  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb.CreateString(cpu_name.c_str());
  flatbuffers::Offset<flatbuffers::String> camera_id_offset =
      fbb.CreateString(camera_id);
  flatbuffers::Offset<flatbuffers::Vector<float>> intrinsics_offset =
      fbb.CreateVector<float>(9u, [&camera_matrix](size_t i) {
        return static_cast<float>(
            reinterpret_cast<double *>(camera_matrix.data)[i]);
      });

  std::optional<uint16_t> camera_number =
      frc971::vision::CameraNumberFromChannel(std::string(camera_channel));

  flatbuffers::Offset<flatbuffers::Vector<float>>
      distortion_coefficients_offset =
          fbb.CreateVector<float>(5u, [&dist_coeffs](size_t i) {
            return static_cast<float>(
                reinterpret_cast<double *>(dist_coeffs.data)[i]);
          });

  calibration::CameraCalibration::Builder camera_calibration_builder(fbb);

  camera_calibration_builder.add_node_name(name_offset);
  camera_calibration_builder.add_team_number(team_number);
  camera_calibration_builder.add_camera_number(camera_number.value());
  camera_calibration_builder.add_camera_id(camera_id_offset);
  camera_calibration_builder.add_reprojection_error(
      static_cast<float>(reprojection_error));
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
    const double reprojection_error = cv::aruco::calibrateCameraCharuco(
        all_charuco_corners_, all_charuco_ids_, charuco_extractor_.board(),
        image_size_, camera_matrix, dist_coeffs, rvecs, tvecs,
        std_deviations_intrinsics, std_deviations_extrinsics, per_view_errors,
        calibration_flags);
    CHECK_LE(reprojection_error, 5.0)
        << ": Reproduction error is bad-- greater than 5 pixels.";
    if (reprojection_error < 2.0) {
      LOG(INFO) << "Reprojection Error is " << reprojection_error;
    } else {
      LOG(WARNING) << "NOTE: Reprojection Error is > 2.0, at "
                   << reprojection_error;
    }

    const aos::realtime_clock::time_point realtime_now =
        aos::realtime_clock::now();
    std::optional<uint16_t> team_number =
        aos::network::team_number_internal::ParsePiOrOrinTeamNumber(hostname_);
    CHECK(team_number) << ": Invalid hostname " << hostname_
                       << ", failed to parse team number";
    aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
        camera_calibration = BuildCalibration(
            camera_matrix, dist_coeffs, realtime_now, cpu_type_.value(),
            cpu_number_.value(), camera_channel_, camera_id_,
            team_number.value(), reprojection_error);
    std::stringstream time_ss;
    time_ss << realtime_now;

    std::optional<uint16_t> camera_number =
        frc971::vision::CameraNumberFromChannel(camera_channel_);
    CHECK(camera_number.has_value());
    std::string calibration_filename =
        CalibrationFilename(calibration_folder_, hostname_, team_number.value(),
                            camera_number.value(), camera_id_, time_ss.str());

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
}  // namespace frc971::vision
