#ifndef y2025_VISION_VISION_UTIL_H_
#define y2025_VISION_VISION_UTIL_H_
#include <map>
#include <string_view>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "opencv2/imgproc.hpp"

#include "y2025/constants/constants_generated.h"

ABSL_DECLARE_FLAG(bool, use_one_orin);

namespace y2025::vision {

// Generate unique colors for each camera
const auto kOrinColors =
    absl::GetFlag(FLAGS_use_one_orin)
        ? std::map<std::string, cv::Scalar>{{"/orin1/camera0",
                                             cv::Scalar(255, 0, 255)},
                                            {"/orin1/camera1",
                                             cv::Scalar(255, 255, 0)}}
        : std::map<std::string, cv::Scalar>{
              {"/orin1/camera0", cv::Scalar(255, 0, 255)},
              {"/orin1/camera1", cv::Scalar(255, 255, 0)},
              {"/imu/camera0", cv::Scalar(0, 255, 255)},
              {"/imu/camera1", cv::Scalar(255, 165, 0)}};

// Structure to store node name (e.g., orin1, imu), number, and a usable string
struct CameraNode {
  std::string node_name;
  int camera_number;

  inline const std::string camera_name() const {
    return "/" + node_name + "/camera" + std::to_string(camera_number);
  }
};

std::vector<CameraNode> CreateNodeList();

std::map<std::string, int> CreateOrderingMap(
    std::vector<CameraNode> &node_list);

const frc971::vision::calibration::CameraCalibration *FindCameraCalibration(
    const y2025::Constants &calibration_data, std::string_view node_name,
    int camera_number);

}  // namespace y2025::vision

#endif  // y2025_VISION_VISION_UTIL_H_
