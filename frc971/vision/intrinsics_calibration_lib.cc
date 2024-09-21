#include "frc971/vision/intrinsics_calibration_lib.h"

#include <filesystem>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "opencv2/core/eigen.hpp"

// NOTE: This will flip any annotations / text that has already been drawn on
// the image (e.g., in charuco_lib.cc)
ABSL_FLAG(bool, flip_image, true,
          "If true, flip the display image so visualization looks correct "
          "during calibration");
ABSL_FLAG(std::string, image_save_path, "",
          "If specified, save out images that are captured for calibration");
ABSL_FLAG(std::string, image_load_path, "",
          "If specified, folder to load images from (image files named "
          "img_######.png");
ABSL_FLAG(
    bool, review_images, false,
    "Whether to review the calibration result (only when reading from files)");
ABSL_FLAG(bool, use_rational_model, true,
          "Whether to use the 8 parameter rational model");

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
      reprojection_error_(std::numeric_limits<double>::max()),
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
  static int invalid_images_saved = 0;
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
    // Require valid poses on loading from disk, so we can visually
    // review once done
    if (!absl::GetFlag(FLAGS_image_load_path).empty()) {
      LOG(FATAL) << "Shouldn't have invalid images in loading from logs";
    }
    VLOG(1) << "Skip because pose is not valid";
    return;
  }

  CHECK(tvecs_eigen.size() == 1)
      << "Charuco board should only return one translational pose.  Returned "
      << tvecs_eigen.size();
  CHECK(rvecs_eigen.size() == 1)
      << "Charuco board should only return one rotational pose. Returned "
      << rvecs_eigen.size();

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

  if (absl::GetFlag(FLAGS_image_load_path) != "") {
    LOG(INFO) << "Loading from disk, so capturing all images";
    store_image = true;
  }

  if ((keystroke & 0xFF) == static_cast<int>('c')) {
    // Protect from hitting capture key but then the image captured is bad
    if (charuco_ids.size() > 15) {
      LOG(INFO) << "Manual capture triggered";
      H_camera_board = prev_H_board_camera_;
      if (!valid) {
        invalid_images_saved++;
      }
      store_image = true;
    } else {
      LOG(INFO) << "Manual attempt rejected due to too few features "
                << charuco_ids.size();
    }
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
      LOG(INFO) << "Storing image #" << all_charuco_corners_.size();
    }
  }

  // TODO<Jim>: Do we really need this?
  if (invalid_images_saved > 0) {
    LOG(INFO) << "Captured " << invalid_images_saved
              << " invalid images out of " << all_charuco_corners_.size();
  }
}

void IntrinsicsCalibration::LoadImages(std::vector<std::string> file_list) {
  file_list_ = file_list;
  for (std::string filename : file_list_) {
    LOG(INFO) << "Loading image " << filename;
    cv::Mat image = cv::imread(filename);
    charuco_extractor_.HandleImage(image, aos::monotonic_clock::now());
  }
}

void IntrinsicsCalibration::LoadImagesFromPath(
    const std::filesystem::path &path) {
  std::vector<std::string> file_list;
  std::filesystem::directory_iterator dir(path);
  for (const auto &entry : dir) {
    if (entry.is_regular_file()) {  // Skip directories
      if (std::regex_match(entry.path().filename().string(),
                           std::regex("img_[0-9]{6}.png"))) {
        file_list.push_back(entry.path().string());
      }
    }
  }

  std::sort(file_list.begin(), file_list.end());
  LOG(INFO) << "Loading " << file_list.size() << " images from "
            << path.string();
  LoadImages(file_list);
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
      distortion_coefficients_offset = fbb.CreateVector<float>(
          dist_coeffs.total(), [&dist_coeffs](size_t i) {
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

void IntrinsicsCalibration::DrawCornersOnImage(cv::Mat image, uint index,
                                               std::vector<cv::Mat> tvecs,
                                               std::vector<cv::Mat> rvecs,
                                               cv::Mat camera_matrix,
                                               cv::Mat dist_coeffs) {
  std::vector<cv::Point3f> board_corners =
      charuco_extractor_.board()->chessboardCorners;
  double square_length = charuco_extractor_.board()->getSquareLength();

  {
    //  Draw my own frame axes, since opencv version has issues
    Eigen::Vector3d tvec, rvec;
    cv::cv2eigen(tvecs[index], tvec);
    cv::cv2eigen(rvecs[index], rvec);
    Eigen::Affine3d T_cam_board =
        Eigen::Translation3d(tvec) *
        Eigen::AngleAxisd(rvec.norm(), rvec / rvec.norm());
    // Project the point T_cam_board to the image using camera_matrix
    // and dist_coeffs
    cv::Mat point_in_cam;
    cv::projectPoints(tvecs[index], cv::Mat::zeros(3, 1, CV_32F),
                      cv::Mat::zeros(3, 1, CV_32F), camera_matrix, dist_coeffs,
                      point_in_cam);
    cv::Point2f point;
    point.x = static_cast<float>(point_in_cam.at<double>(0, 0));
    point.y = static_cast<float>(point_in_cam.at<double>(0, 1));
    // Draw a circle using T_cam_board
    cv::circle(image, point, 10, cv::Scalar(0, 255, 0), 2);
    for (uint i = 0; i < 3; i++) {
      Eigen::Vector3d eye_col = Eigen::Matrix3d::Identity().col(i);
      Eigen::Vector3d ray = T_cam_board * (square_length * eye_col);

      cv::Scalar color =
          cv::Scalar((int)(eye_col(2, 0) * 255), (int)(eye_col(1, 0) * 255),
                     (int)(eye_col(0, 0) * 255));
      cv::Mat ray_cv;
      cv::eigen2cv(ray, ray_cv);
      cv::projectPoints(ray_cv, cv::Mat::zeros(3, 1, CV_32F),
                        cv::Mat::zeros(3, 1, CV_32F), camera_matrix,
                        dist_coeffs, point_in_cam);
      cv::Point2f end_point;
      end_point.x = (float)point_in_cam.at<double>(0, 0);
      end_point.y = (float)point_in_cam.at<double>(0, 1);
      cv::line(image, point, end_point, color, 2);
    }

    // Draw circles around locations of all the corners of the board
    for (cv::Point3f corner : board_corners) {
      Eigen::Vector3d point;
      point << corner.x, corner.y, corner.z;
      // Map charuco corner (3D) point to have camera as origin
      Eigen::Vector3d transformed_point = T_cam_board * point;
      cv::Mat t_point_cv;
      cv::eigen2cv(transformed_point, t_point_cv);
      // Project point into camera view using our camera intrinsics
      cv::projectPoints(t_point_cv, cv::Mat::zeros(3, 1, CV_32F),
                        cv::Mat::zeros(3, 1, CV_32F), camera_matrix,
                        dist_coeffs, point_in_cam);
      cv::Point2f point_2f;
      point_2f.x = (float)point_in_cam.at<double>(0, 0);
      point_2f.y = (float)point_in_cam.at<double>(0, 1);
      cv::circle(image, point_2f, 10, cv::Scalar(0, 0, 255), 2);
    }

    // Then, draw over them with green circles showing our matches
    for (uint j = 0; j < all_charuco_ids_[index].size(); j++) {
      cv::Point3f corner = board_corners[all_charuco_ids_[index][j]];
      Eigen::Vector3d point;
      point << corner.x, corner.y, corner.z;
      Eigen::Vector3d transformed_point = T_cam_board * point;
      cv::Mat t_point_cv;
      cv::eigen2cv(transformed_point, t_point_cv);
      cv::projectPoints(t_point_cv, cv::Mat::zeros(3, 1, CV_32F),
                        cv::Mat::zeros(3, 1, CV_32F), camera_matrix,
                        dist_coeffs, point_in_cam);
      cv::Point2f point_2f;
      point_2f.x = (float)point_in_cam.at<double>(0, 0);
      point_2f.y = (float)point_in_cam.at<double>(0, 1);
      cv::circle(image, point_2f, 10, cv::Scalar(0, 255, 0), 2);
    }
  }
}

void IntrinsicsCalibration::MaybeCalibrate() {
  // TODO: This number should depend on coarse vs. fine pattern
  // Maybe just on total # of ids found, not just images
  if (all_charuco_ids_.size() >= 50) {
    int total_num_ids = 0;
    for (auto charuco_ids : all_charuco_ids_) {
      total_num_ids += charuco_ids.size();
    }
    LOG(INFO) << "Beginning calibration on " << all_charuco_ids_.size()
              << " images, with " << total_num_ids << " ids total";

    if (camera_mat_.empty()) {
      camera_mat_ = charuco_extractor_.camera_matrix();
    }
    if (dist_coeffs_.empty()) {
      dist_coeffs_ = charuco_extractor_.dist_coeffs();
    }
    cv::Mat std_deviations_intrinsics, std_deviations_extrinsics,
        per_view_errors;
    // Providing an initial guess of the intrinsics is very useful, even if it's
    // just approximate or from an old camera
    int calibration_flags = cv::CALIB_USE_INTRINSIC_GUESS;
    if (absl::GetFlag(FLAGS_use_rational_model)) {
      LOG(INFO) << "Using rational (8-parameter) model";
      calibration_flags |= cv::CALIB_RATIONAL_MODEL;
    }
    // Found that using at least 100 iterations helped get convergence to
    // correct values
    cv::TermCriteria term_crit = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 1000, 1e-9);
    reprojection_error_ = cv::aruco::calibrateCameraCharuco(
        all_charuco_corners_, all_charuco_ids_, charuco_extractor_.board(),
        image_size_, camera_mat_, dist_coeffs_, rvecs_, tvecs_,
        std_deviations_intrinsics, std_deviations_extrinsics, per_view_errors,
        calibration_flags, term_crit);

    CHECK_LE(reprojection_error_, 5.0)
        << ": Reproduction error is bad-- greater than 5 pixels.";
    if (reprojection_error_ < 1.0) {
      LOG(INFO) << "Reprojection Error is " << reprojection_error_;
    } else {
      LOG(WARNING) << "NOTE: Reprojection Error is > 1.0, at "
                   << reprojection_error_;
    }

    const aos::realtime_clock::time_point realtime_now =
        aos::realtime_clock::now();
    std::optional<uint16_t> team_number =
        aos::network::team_number_internal::ParsePiOrOrinTeamNumber(hostname_);
    CHECK(team_number) << ": Invalid hostname " << hostname_
                       << ", failed to parse team number";
    std::optional<std::string_view> cpu_type =
        aos::network::ParsePiOrOrin(hostname_);
    CHECK(cpu_type) << ": Invalid cpu_type from" << hostname_
                    << ", failed to parse cpu type";
    std::optional<uint16_t> cpu_number =
        aos::network::ParsePiOrOrinNumber(hostname_);
    std::string node_name =
        std::string(cpu_type.value()) +
        (cpu_number ? std::to_string(cpu_number.value()) : "");
    // Hack around naming scheme for imu == orin2
    if (node_name == "orin2") {
      node_name = "imu";
    }
    aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
        camera_calibration = BuildCalibration(
            camera_mat_, dist_coeffs_, realtime_now, cpu_type_.value(),
            cpu_number_.value(), camera_channel_, camera_id_,
            team_number.value(), reprojection_error_);
    std::stringstream time_ss;
    time_ss << realtime_now;

    std::optional<uint16_t> camera_number =
        frc971::vision::CameraNumberFromChannel(camera_channel_);
    CHECK(camera_number.has_value());
    std::string calibration_filename =
        CalibrationFilename(calibration_folder_, node_name, team_number.value(),
                            camera_number.value(), camera_id_, time_ss.str());

    LOG(INFO) << calibration_filename << " -> "
              << aos::FlatbufferToJson(camera_calibration,
                                       {.multi_line = true});
    aos::util::WriteStringToFileOrDie(
        calibration_filename,
        aos::FlatbufferToJson(camera_calibration, {.multi_line = true}));

    // If desired, review the images (requires we're loading from disk);
    if (absl::GetFlag(FLAGS_review_images) &&
        !absl::GetFlag(FLAGS_image_load_path).empty()) {
      uint index = 0;
      for (std::string filename : file_list_) {
        LOG(INFO) << "Loading file " << filename;
        cv::Mat image = cv::imread(filename);
        DrawCornersOnImage(image, index, tvecs_, rvecs_, camera_mat_,
                           dist_coeffs_);
        cv::imshow("Image review", image);
        cv::waitKey(0);
        index++;
      }
    }
  } else {
    LOG(INFO) << "Skipping calibration due to not enough images.";
  }
}
}  // namespace frc971::vision
