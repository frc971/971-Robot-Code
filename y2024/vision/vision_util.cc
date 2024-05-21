#include "y2024/vision/vision_util.h"

#include "glog/logging.h"

namespace y2024::vision {

// Store a list of ordered cameras as you progress around the robot/box of orins
std::vector<CameraNode> CreateNodeList() {
  std::vector<CameraNode> list;

  list.push_back({.node_name = "imu", .camera_number = 0});
  list.push_back({.node_name = "imu", .camera_number = 1});
  list.push_back({.node_name = "orin1", .camera_number = 1});
  list.push_back({.node_name = "orin1", .camera_number = 0});

  return list;
}

// From the node_list, create a numbering scheme from 0 to 3
std::map<std::string, int> CreateOrderingMap(
    std::vector<CameraNode> &node_list) {
  std::map<std::string, int> map;

  for (uint i = 0; i < node_list.size(); i++) {
    map.insert({node_list.at(i).camera_name(), i});
  }

  return map;
}

const frc971::vision::calibration::CameraCalibration *FindCameraCalibration(
    const y2024::Constants &calibration_data, std::string_view node_name,
    int camera_number) {
  CHECK(calibration_data.robot()->has_cameras());
  for (const y2024::CameraConfiguration *candidate :
       *calibration_data.robot()->cameras()) {
    CHECK(candidate->has_calibration());
    if (candidate->calibration()->node_name()->string_view() != node_name ||
        candidate->calibration()->camera_number() != camera_number) {
      continue;
    }
    return candidate->calibration();
  }
  LOG(FATAL) << ": Failed to find camera calibration for " << node_name
             << " and camera number " << camera_number;
}

}  // namespace y2024::vision
