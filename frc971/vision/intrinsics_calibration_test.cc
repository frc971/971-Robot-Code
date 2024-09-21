#include <filesystem>
#include <random>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"
#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/shm_event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/testing/path.h"
#include "aos/testing/random_seed.h"
#include "aos/util/math.h"
#include "frc971/vision/intrinsics_calibration_lib.h"

namespace frc971::vision {
aos::distributed_clock::time_point TimeInMs(size_t ms) {
  return aos::distributed_clock::time_point(std::chrono::milliseconds(ms));
}
namespace fs = std::filesystem;

void CheckAgainstKnownCalibration(IntrinsicsCalibration &calibrator) {
  // Retrieve the original camera intrinsics
  cv::Mat camera_mat = calibrator.GetCharucoExtractor().camera_matrix();
  cv::Mat dist_coeffs = calibrator.GetCharucoExtractor().dist_coeffs();

  // Retrieve the computed camera intrinsics
  cv::Mat calculated_camera_mat = calibrator.GetCameraMatrix();
  cv::Mat calculated_dist_coeffs = calibrator.GetDistortionCoefficients();

  // Check that our estimates are close to the correct values
  cv::Mat diff_camera_mat;
  cv::Mat diff_dist_coeffs;
  // Have to force add into CV_32F, since calculated_XXX is CV_64F
  cv::add(camera_mat, -calculated_camera_mat, diff_camera_mat, cv::noArray(),
          CV_32F);
  cv::add(dist_coeffs, -calculated_dist_coeffs, diff_dist_coeffs, cv::noArray(),
          CV_32F);

  VLOG(1) << "Norm of diffs is " << cv::norm(diff_camera_mat) << " and "
          << cv::norm(diff_dist_coeffs);

  const double kDiffThreshold = 1e-3;
  EXPECT_NEAR(cv::norm(diff_camera_mat), 0.0, kDiffThreshold);
  EXPECT_NEAR(cv::norm(diff_dist_coeffs), 0.0, kDiffThreshold);
}

// With the specified camera calibration file, project a series of boards
// (using their corner points) into the image plane at different poses, and
// generate and check the intrinsic calibration that is computed based on those
// points.
//
// Allow for different intrinsics (camera matrix and distortion coefficients) to
// be used as initial conditions for the solver.
void RunIntrinsicFromPoints(std::string calib_filename,
                            cv::Mat camera_mat_init = cv::Mat(),
                            cv::Mat dist_coeffs_init = cv::Mat()) {
  absl::SetFlag(&FLAGS_override_hostname, "orin-971-1");
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("y2024/aos_config.stripped.json");
  aos::ShmEventLoop event_loop(&config.message());
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();
  IntrinsicsCalibration calibrator(&event_loop, "orin-971-1", "/orin1/camera0",
                                   "24-00", calib_filename.c_str(), false,
                                   "/tmp", exit_handle.get());
  // We disable visualize for this test, so we don't try to draw things
  absl::SetFlag(&FLAGS_visualize, false);

  cv::Mat zero_vec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Size image_size = cv::Size(1456, 1088);
  cv::Ptr<cv::aruco::CharucoBoard> board =
      calibrator.GetCharucoExtractor().board();

  // Retrieve the camera parameters from the config
  cv::Mat camera_mat = calibrator.GetCharucoExtractor().camera_matrix();
  cv::Mat dist_coeffs = calibrator.GetCharucoExtractor().dist_coeffs();

  // If we're not given initial camera parameters, use the actual camera
  // parameters as initial guesses
  if (camera_mat_init.empty()) {
    camera_mat_init = camera_mat;
  }
  // And same with distortion coefficients
  if (dist_coeffs_init.empty()) {
    dist_coeffs_init = dist_coeffs;
  }
  calibrator.SetCameraMatrix(camera_mat_init);
  calibrator.SetDistortionCoefficients(dist_coeffs_init);

  if (dist_coeffs.size().height == 5) {
    LOG(INFO) << "Using 5 parameter traditional model";
    absl::SetFlag(&FLAGS_use_rational_model, false);
  } else {
    LOG(INFO) << "Using 8 parameter rational model";
    absl::SetFlag(&FLAGS_use_rational_model, true);
  }

  // Choose reasonable values for starting point of board
  Eigen::Matrix3f R_c_1;
  R_c_1 = Eigen::AngleAxis<float>(0.0, Eigen::Vector3f::UnitX());
  Eigen::Vector3f T_c_1(-0.3, 0.3, 0.3);
  Eigen::Affine3f H_c_1 = Eigen::Translation3f(T_c_1) * R_c_1;

  // Storage to collect all the projected corners and ids
  std::vector<std::vector<int>> all_corner_ids;
  std::vector<std::vector<cv::Point2f>> all_corners;

  bool should_break = false;
  // Iterate through a bunch of poses so that we present the board at enough
  // views to cover the image plane with corner points
  for (float theta_x = 0.0; theta_x <= M_PI / 6 && !should_break;
       theta_x += M_PI / 8) {
    for (float theta_y = 0.0; theta_y <= M_PI / 6 && !should_break;
         theta_y += M_PI / 8) {
      for (float depth = 0.6; depth >= 0.0 && !should_break; depth -= 0.2) {
        for (float x = -0.5; x <= 0.5 && !should_break; x += 0.5) {
          for (float y = -1.0; y <= 0.0 && !should_break; y += 0.5) {
            // Go until we've captured enough samples
            if (all_corner_ids.size() > 75) {
              int total_num_ids = 0;
              for (auto corner_ids : all_corner_ids) {
                total_num_ids += corner_ids.size();
              }
              LOG(INFO) << "Got enough board captures: "
                        << all_corner_ids.size() << " with " << total_num_ids
                        << " corners";
              should_break = true;
              break;
            }

            // Create the transform from starting pose (1) to target pose (2)
            Eigen::Matrix3f R_1_2;
            R_1_2 = Eigen::AngleAxis<float>(theta_x, Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxis<float>(theta_y, Eigen::Vector3f::UnitY());
            Eigen::Vector3f T_1_2(x, y, depth);
            Eigen::Affine3f H_1_2 = Eigen::Translation3f(T_1_2) * R_1_2;
            Eigen::Affine3f H_c_2 = H_c_1 * H_1_2;

            // Create set of corners and ids that are projected into this view
            // and are within our image boundaries
            std::vector<int> corner_id_list;
            std::vector<cv::Point2f> corner_list;

            cv::Mat point_viz = cv::Mat::zeros(image_size, CV_8UC3);
            for (size_t i = 0; i < board->chessboardCorners.size(); i++) {
              // Transform the ith corner into its new point location
              Eigen::Vector3f point_eigen;
              cv::cv2eigen(cv::Mat(board->chessboardCorners[i]), point_eigen);
              Eigen::Vector3f new_point =
                  (H_c_2 * Eigen::Translation3f(point_eigen)).translation();
              cv::Mat new_point_cv;
              cv::eigen2cv(new_point, new_point_cv);
              // Project the new 3D location of the point to th eimage plane
              cv::Mat new_points_proj(2, 1, CV_32F);
              cv::projectPoints(new_point_cv, zero_vec, zero_vec, camera_mat,
                                dist_coeffs, new_points_proj);
              float u = new_points_proj.at<float>(0, 0);
              float v = new_points_proj.at<float>(0, 1);
              if (u >= 0 && u <= image_size.width && v >= 0 &&
                  v <= image_size.height) {
                int id = static_cast<int>(i);
                cv::Point2f corner_proj(u, v);
                corner_id_list.push_back(id);
                corner_list.push_back(corner_proj);
                cv::circle(point_viz, corner_proj, 3, cv::Scalar(0, 255, 0));
              }
            }
            // For visualizing the corners that are generated
            // cv::imshow("point_viz", point_viz);
            // cv::waitKey(0);

            // Require that we have at least 15 visible corners in order to push
            // them back on our stack of valid projected corners
            if (corner_id_list.size() > 15) {
              all_corner_ids.push_back(corner_id_list);
              all_corners.push_back(corner_list);
            }
          }
        }
      }
    }
  }

  // Set the corners in our intrinsic calibration routine
  calibrator.SetCharucoIds(all_corner_ids);
  calibrator.SetCharucoCorners(all_corners);

  cv::Mat blank_image = cv::Mat::zeros(image_size, CV_8UC3);
  // Push image size by sending a blank image of the target size
  calibrator.GetCharucoExtractor().HandleImage(blank_image,
                                               aos::monotonic_clock::now());
  // Run the calibration
  calibrator.MaybeCalibrate();

  {
    // For visualizing all points that have been captured
    cv::Mat all_point_viz = cv::Mat::zeros(image_size, CV_8UC3);
    // Draw a small circle for each corner
    for (uint j = 0; j < all_corners.size(); j++) {
      for (uint i = 0; i < all_corners[j].size(); i++) {
        cv::Point2f point = all_corners[j][i];
        cv::circle(all_point_viz, point, 3, cv::Scalar(255, 255, 255));
      }
    }
    //  cv::imshow("all_point_viz", all_point_viz);
    //  cv::waitKey(0);
  }

  {
    // Iterate through the points and visualize versus computed
    for (uint i = 0; i < all_corners.size(); i++) {
      cv::Mat display_image = cv::Mat::zeros(image_size, CV_8UC3);
      // Draw original points
      for (uint j = 0; j < all_corners[i].size(); j++) {
        cv::Point2f point = all_corners[i][j];
        // point = point;
        cv::circle(display_image, point, 3, cv::Scalar(255, 255, 255));
      }
      // Retrieve the computed camera intrinsics
      cv::Mat calculated_camera_mat = calibrator.GetCameraMatrix();
      cv::Mat calculated_dist_coeffs = calibrator.GetDistortionCoefficients();

      // Draw projected points with new calibration
      calibrator.DrawCornersOnImage(
          display_image, i, calibrator.GetTVecs(), calibrator.GetRVecs(),
          calculated_camera_mat, calculated_dist_coeffs);
      //    cv::imshow("compare points", display_image);
      //    cv::waitKey(30);
    }
  }

  CheckAgainstKnownCalibration(calibrator);
  // Reprojection error should be nearly exact
  EXPECT_NEAR(calibrator.GetReprojectionError(), 0.0, 0.001);
}

TEST(IntrinsicCalculationTest, SamplingProjectedPoints) {
  // Run using old 5 parameter model (which requires distortion coefficients
  // to be zero-- otherwise our reprojections don't work near the corners of
  // the image)
  RunIntrinsicFromPoints(std::string(
      "y2024/constants/calib_files/calibration_orin-971-1_cam-24-00.json"));

  // Run the new 8 parameter rational model
  RunIntrinsicFromPoints(
      std::string("y2024/constants/calib_files/"
                  "calibration_orin-971-1_cam-24-00_8parameter.json"));

  // Run the 8 parameter model using rough values for the camera matrix and
  // distortion coeficients to check it converges properly
  cv::Mat camera_mat_init = cv::Mat::eye(3, 3, CV_32F);
  camera_mat_init.at<float>(0, 0) = 650.0;
  camera_mat_init.at<float>(1, 1) = 650.0;
  camera_mat_init.at<float>(0, 2) = 1456 / 2.0;
  camera_mat_init.at<float>(1, 2) = 1088 / 2.0;
  cv::Mat dist_coeffs_init = cv::Mat::zeros(8, 1, CV_32F);

  RunIntrinsicFromPoints(
      std::string("y2024/constants/calib_files/"
                  "calibration_orin-971-1_cam-24-00_8parameter.json"),
      camera_mat_init, dist_coeffs_init);
}

// Test intrinsic calibration when loading from a series of images from disk
TEST(IntrinsicCalculationTest, ImagePlayback) {
  absl::SetFlag(&FLAGS_override_hostname, "orin-971-1");
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("y2024/aos_config.stripped.json");
  aos::ShmEventLoop event_loop(&config.message());
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();

  // Use the 8 parameter intrinsic calibraiotn model
  absl::SetFlag(&FLAGS_use_rational_model, true);

  IntrinsicsCalibration calibrator(
      &event_loop, "orin-971-1", "/orin1/camera0", "24-00",
      "y2024/constants/calib_files/"
      "calibration_orin-971-1_cam-24-00_8parameter.json",
      false, "/tmp", exit_handle.get());

  // We disable visualize for this test, so we don't try to draw things
  absl::SetFlag(&FLAGS_visualize, false);

  // Run the calibration from files
  std::filesystem::path test_images_path(
      "external/intrinsic_calibration_test_images");
  LOG(INFO) << "Running intrinsics from disk from path: "
            << test_images_path.string();
  absl::SetFlag(&FLAGS_image_load_path, test_images_path.string());
  calibrator.LoadImagesFromPath(test_images_path);
  calibrator.MaybeCalibrate();

  CheckAgainstKnownCalibration(calibrator);
  // Validate that reprojection error is < 1
  EXPECT_LT(calibrator.GetReprojectionError(), 1.0);
  EXPECT_GT(calibrator.NumCaptures(), 50);
}
}  // namespace frc971::vision
