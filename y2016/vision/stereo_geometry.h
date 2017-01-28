#ifndef Y2016_VISION_STEREO_GEOMETRY_H_
#define Y2016_VISION_STEREO_GEOMETRY_H_

#include <string>

#include "aos/common/logging/logging.h"
#include "aos/vision/math/vector.h"

#include "y2016/vision/calibration.pb.h"

namespace y2016 {
namespace vision {

// Returns the contents of the calibration file which are embedded into the
// code.
CalibrationFile EmbeddedCalibrationFile();

// Returns the embedded Calibration for the given robot_name or LOG(FATAL)s.
Calibration FindCalibrationForRobotOrDie(
    const ::std::string &robot_name, const CalibrationFile &calibration_file);

class StereoGeometry {
 public:
  StereoGeometry(const ::std::string &robot_name)
      : calibration_(FindCalibrationForRobotOrDie(robot_name,
                                                  EmbeddedCalibrationFile())) {}

  // Returns the forward world distance in meters corresponding to the given
  // distance in pixels between the two cameras.
  double PixelToWorldDistance(double pixel_distance) {
    return (calibration_.center_center_dist() *
            calibration_.camera_image_width() * calibration_.focal_length() /
            (pixel_distance + calibration_.center_center_skew())) +
           calibration_.measure_camera_offset();
  }

  // Converts locations on both cameras to the distance between the locations
  // and both angles.
  // Returns (via pointers) the distance to the target in meters, the horizontal
  // angle to the target in radians, and the vertical angle to the target in
  // radians.
  // TODO(Brian): Figure out which way is positive on the angles.
  void Process(const ::aos::vision::Vector<2> &center0,
               const ::aos::vision::Vector<2> &center1, double *distance,
               double *horizontal_angle, double *vertical_angle) {
    *distance = PixelToWorldDistance(center1.x() - center0.x());
    const double average_x = (center0.x() + center1.x()) / 2.0 -
                             calibration_.camera_image_width() / 2.0;
    const double average_y = (center0.y() + center1.y()) / 2.0 -
                             calibration_.camera_image_height() / 2.0;

    *horizontal_angle =
        std::atan2(average_x, calibration_.camera_image_width() *
                                  calibration_.focal_length());
    *vertical_angle = std::atan2(average_y, calibration_.camera_image_width() *
                                                calibration_.focal_length());
  }

  const Calibration &calibration() const { return calibration_; }

 private:
  Calibration calibration_;
};

}  // namespace vision
}  // namespace y2016

#endif  // Y2016_VISION_STEREO_GEOMETRY_H_
