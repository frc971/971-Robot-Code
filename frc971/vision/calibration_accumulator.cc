#include "frc971/vision/calibration_accumulator.h"

#include <algorithm>
#include <limits>
#include <opencv2/highgui/highgui.hpp>

#include "Eigen/Dense"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "frc971/control_loops/quaternion_utils.h"
#include "frc971/vision/charuco_lib.h"
#include "frc971/wpilib/imu_batch_generated.h"

DEFINE_bool(display_undistorted, false,
            "If true, display the undistorted image.");
DEFINE_string(save_path, "", "Where to store annotated images");
DEFINE_bool(save_valid_only, false,
            "If true, only save images with valid pose estimates");

namespace frc971 {
namespace vision {
using aos::distributed_clock;
using aos::monotonic_clock;
namespace chrono = std::chrono;

constexpr double kG = 9.807;

void CalibrationData::AddCameraPose(
    distributed_clock::time_point distributed_now, Eigen::Vector3d rvec,
    Eigen::Vector3d tvec) {
  // Always start with IMU (or turret) reading...
  // Note, we may not have a turret, so need to handle that case
  // If we later get a turret point, then we handle removal of camera points in
  // AddTurret
  if ((!imu_points_.empty() && imu_points_[0].first < distributed_now) &&
      (turret_points_.empty() || turret_points_[0].first < distributed_now)) {
    rot_trans_points_.emplace_back(distributed_now, std::make_pair(rvec, tvec));
  }
}

void CalibrationData::AddImu(distributed_clock::time_point distributed_now,
                             Eigen::Vector3d gyro, Eigen::Vector3d accel) {
  double zero_threshold = 1e-12;
  // We seem to be getting 0 readings on IMU, so ignore for now
  // TODO<Jim>: I think this has been resolved in HandleIMU, but want to leave
  // this here just in case there are other ways this could happen
  if ((fabs(accel(0)) < zero_threshold) && (fabs(accel(1)) < zero_threshold) &&
      (fabs(accel(2)) < zero_threshold)) {
    LOG(FATAL) << "Ignoring zero value from IMU accelerometer: " << accel
               << " (gyro is " << gyro << ")";
  } else {
    imu_points_.emplace_back(distributed_now, std::make_pair(gyro, accel));
  }
}

void CalibrationData::AddTurret(
    aos::distributed_clock::time_point distributed_now, Eigen::Vector2d state) {
  // We want the turret to be known too when solving.  But, we don't know if
  // we are going to have a turret until we get the first reading.  In that
  // case, blow away any camera readings from before.
  // NOTE: Since the IMU motion is independent of the turret position, we don't
  // need to remove the IMU readings before the turret
  if (turret_points_.empty()) {
    while (!rot_trans_points_.empty() &&
           rot_trans_points_[0].first < distributed_now) {
      LOG(INFO) << "Erasing, distributed " << distributed_now;
      rot_trans_points_.erase(rot_trans_points_.begin());
    }
  }
  turret_points_.emplace_back(distributed_now, state);
}

void CalibrationData::ReviewData(CalibrationDataObserver *observer) const {
  size_t next_camera_point = 0;
  size_t next_imu_point = 0;
  size_t next_turret_point = 0;

  // Just go until one of the data streams runs out.  We lose a few points, but
  // it makes the logic much easier
  while (
      next_camera_point != rot_trans_points_.size() &&
      next_imu_point != imu_points_.size() &&
      (turret_points_.empty() || next_turret_point != turret_points_.size())) {
    // If camera_point is next, update it
    if ((rot_trans_points_[next_camera_point].first <=
         imu_points_[next_imu_point].first) &&
        (turret_points_.empty() ||
         (rot_trans_points_[next_camera_point].first <=
          turret_points_[next_turret_point].first))) {
      // Camera!
      observer->UpdateCamera(rot_trans_points_[next_camera_point].first,
                             rot_trans_points_[next_camera_point].second);
      ++next_camera_point;
    } else {
      // If it's not the camera, check if IMU is next
      if (turret_points_.empty() || (imu_points_[next_imu_point].first <=
                                     turret_points_[next_turret_point].first)) {
        // IMU!
        observer->UpdateIMU(imu_points_[next_imu_point].first,
                            imu_points_[next_imu_point].second);
        ++next_imu_point;
      } else {
        // If it's not IMU or camera, and turret_points is not empty, it must be
        // the turret!
        observer->UpdateTurret(turret_points_[next_turret_point].first,
                               turret_points_[next_turret_point].second);
        ++next_turret_point;
      }
    }
  }
}

Calibration::Calibration(aos::SimulatedEventLoopFactory *event_loop_factory,
                         aos::EventLoop *image_event_loop,
                         aos::EventLoop *imu_event_loop, std::string_view pi,
                         CalibrationData *data)
    : image_event_loop_(image_event_loop),
      image_factory_(event_loop_factory->GetNodeEventLoopFactory(
          image_event_loop_->node())),
      imu_event_loop_(imu_event_loop),
      imu_factory_(
          event_loop_factory->GetNodeEventLoopFactory(imu_event_loop_->node())),
      charuco_extractor_(
          image_event_loop_, pi,
          [this](cv::Mat rgb_image, monotonic_clock::time_point eof,
                 std::vector<cv::Vec4i> charuco_ids,
                 std::vector<std::vector<cv::Point2f>> charuco_corners,
                 bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                 std::vector<Eigen::Vector3d> tvecs_eigen) {
            HandleCharuco(rgb_image, eof, charuco_ids, charuco_corners, valid,
                          rvecs_eigen, tvecs_eigen);
          }),
      image_callback_(
          image_event_loop_,
          absl::StrCat("/pi",
                       std::to_string(aos::network::ParsePiNumber(pi).value()),
                       "/camera"),
          [this](cv::Mat rgb_image, const monotonic_clock::time_point eof) {
            charuco_extractor_.HandleImage(rgb_image, eof);
          }),
      data_(data) {
  imu_factory_->OnShutdown([]() { cv::destroyAllWindows(); });

  // Check for IMUValuesBatch topic on both /localizer and /drivetrain channels,
  // since both are valid/possible
  std::string imu_channel;
  if (imu_event_loop->HasChannel<frc971::IMUValuesBatch>("/localizer")) {
    imu_channel = "/localizer";
  } else if (imu_event_loop->HasChannel<frc971::IMUValuesBatch>(
                 "/drivetrain")) {
    imu_channel = "/drivetrain";
  } else {
    LOG(FATAL) << "Couldn't find channel with IMU data for either localizer or "
                  "drivtrain";
  }

  VLOG(2) << "Listening for " << frc971::IMUValuesBatch::GetFullyQualifiedName()
          << " on channel: " << imu_channel;

  imu_event_loop_->MakeWatcher(
      imu_channel, [this](const frc971::IMUValuesBatch &imu) {
        if (!imu.has_readings()) {
          return;
        }
        for (const frc971::IMUValues *value : *imu.readings()) {
          HandleIMU(value);
        }
      });
}

void Calibration::HandleCharuco(
    cv::Mat rgb_image, const monotonic_clock::time_point eof,
    std::vector<cv::Vec4i> /*charuco_ids*/,
    std::vector<std::vector<cv::Point2f>> /*charuco_corners*/, bool valid,
    std::vector<Eigen::Vector3d> rvecs_eigen,
    std::vector<Eigen::Vector3d> tvecs_eigen) {
  if (valid) {
    CHECK(rvecs_eigen.size() > 0) << "Require at least one target detected";
    // We only use one (the first) target detected for calibration
    data_->AddCameraPose(image_factory_->ToDistributedClock(eof),
                         rvecs_eigen[0], tvecs_eigen[0]);

    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]",
                             "[", "]");

    const double age_double =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            image_event_loop_->monotonic_now() - eof)
            .count();
    VLOG(1) << std::fixed << std::setprecision(6) << "Age: " << age_double
            << ", Pose is R:" << rvecs_eigen[0].transpose().format(HeavyFmt)
            << "\nT:" << tvecs_eigen[0].transpose().format(HeavyFmt);
  }

  if (FLAGS_visualize) {
    if (FLAGS_display_undistorted) {
      const cv::Size image_size(rgb_image.cols, rgb_image.rows);
      cv::Mat undistorted_rgb_image(image_size, CV_8UC3);
      cv::undistort(rgb_image, undistorted_rgb_image,
                    charuco_extractor_.camera_matrix(),
                    charuco_extractor_.dist_coeffs());

      cv::imshow("Display undist", undistorted_rgb_image);
    }

    cv::imshow("Display", rgb_image);
    cv::waitKey(1);
  }

  if (FLAGS_save_path != "") {
    if (!FLAGS_save_valid_only || valid) {
      static int img_count = 0;
      std::string image_name = absl::StrFormat("/img_%06d.png", img_count);
      std::string path = FLAGS_save_path + image_name;
      VLOG(2) << "Saving image to " << path;
      cv::imwrite(path, rgb_image);
      img_count++;
    }
  }
}

void Calibration::HandleIMU(const frc971::IMUValues *imu) {
  // Need to check for valid values, since we sometimes don't get them
  if (!imu->has_gyro_x() || !imu->has_gyro_y() || !imu->has_gyro_z() ||
      !imu->has_accelerometer_x() || !imu->has_accelerometer_y() ||
      !imu->has_accelerometer_z()) {
    return;
  }

  VLOG(2) << "IMU " << imu;
  imu->UnPackTo(&last_value_);
  Eigen::Vector3d gyro(last_value_.gyro_x, last_value_.gyro_y,
                       last_value_.gyro_z);
  Eigen::Vector3d accel(last_value_.accelerometer_x,
                        last_value_.accelerometer_y,
                        last_value_.accelerometer_z);

  data_->AddImu(imu_factory_->ToDistributedClock(monotonic_clock::time_point(
                    chrono::nanoseconds(imu->monotonic_timestamp_ns()))),
                gyro, accel * kG);
}

}  // namespace vision
}  // namespace frc971
