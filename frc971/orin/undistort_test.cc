#include <filesystem>

#include "gtest/gtest.h"

#include "aos/init.h"
#include "apriltag.h"
#include "frc971/vision/calibration_generated.h"
#include "frc971/vision/charuco_lib.h"
#include "gpu_apriltag.h"

using namespace frc971::vision;
using namespace frc971::apriltag;

void TestCalibrationFile(std::filesystem::path base_intrinsics_file) {
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      base_intrinsics =
          aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
              base_intrinsics_file.string());

  CameraCalibration calibration(&base_intrinsics.message());

  CameraMatrix camera_matrix(GetCameraMatrix(&base_intrinsics.message()));
  DistCoeffs dist_coeffs(GetDistCoeffs(&base_intrinsics.message()));

  for (double u = 0; u < 1456; u += 1.0) {
    for (double v = 0; v < 1088; v += 1.0) {
      double u_d = u, v_d = v;
      GpuDetector::ReDistort(&u_d, &v_d, &camera_matrix, &dist_coeffs);
      GpuDetector::UnDistort(&u_d, &v_d, &camera_matrix, &dist_coeffs);

      EXPECT_NEAR(u, u_d, 1e-3);
      EXPECT_NEAR(v, v_d, 1e-3);
    }
  }
}

// Given a camera matrix and distortion coefficients, test out our (Re)distort
// and undistort routines
TEST(UndistortTest, DistortUndistort) {
  // Test against default zero distortion 5 parameter model
  std::string base_intrinsics_file =
      "y2024/constants/calib_files/calibration_orin-971-1_cam-24-00.json";
  LOG(INFO) << "Test against default zero distortion 5 parameter model";
  TestCalibrationFile(base_intrinsics_file);

  // Test against regular camera distortion 5 parameter model
  base_intrinsics_file =
      "y2024/constants/calib_files/"
      "calibration_orin1-971-0_cam-24-06_2024-03-24_14-54-27.json";
  LOG(INFO) << "Test against regular distortion 5 parameter model";
  TestCalibrationFile(base_intrinsics_file);

  // Test against regular camera distortion 5 parameter model
  base_intrinsics_file =
      "y2024/constants/calib_files/"
      "calibration_orin-971-1_cam-24-00_8parameter.json";
  LOG(INFO) << "Test against 8 parameter model";
  TestCalibrationFile(base_intrinsics_file);
}
