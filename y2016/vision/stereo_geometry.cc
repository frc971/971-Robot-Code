#include "y2016/vision/stereo_geometry.h"

namespace y2016 {
namespace vision {

Calibration FindCalibrationForRobotOrDie(
    const ::std::string &robot_name, const CalibrationFile &calibration_file) {
  for (const RobotCalibration &calibration : calibration_file.calibration()) {
    if (calibration.robot() == robot_name) {
      return calibration.calibration();
    }
  }
  LOG(FATAL, "no calibration for %s found in %s\n", robot_name.c_str(),
      calibration_file.ShortDebugString().c_str());
}

}  // namespace vision
}  // namespace y2016
