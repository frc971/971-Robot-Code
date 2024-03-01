#include "y2024/vision/vision_util.h"

#include "glog/logging.h"

namespace y2024::vision {

const frc971::vision::calibration::CameraCalibration *FindCameraCalibration(
    const y2024::Constants &calibration_data, std::string_view node_name,
    int camera_number) {
  CHECK(calibration_data.has_cameras());
  for (const y2024::CameraConfiguration *candidate :
       *calibration_data.cameras()) {
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
