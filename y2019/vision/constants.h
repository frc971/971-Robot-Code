#ifndef _Y2019_VISION_CONSTANTS_H_
#define _Y2019_VISION_CONSTANTS_H_

#include <math.h>
#include <array>
#include <string>

namespace y2019 {
namespace vision {

// Position of the idealized camera in 3d space.
struct CameraGeometry {
  static constexpr size_t kNumParams = 4;
  // In Meters from floor under imu center.
  std::array<double, 3> location{{0, 0, 0}};
  double heading = 0.0;

  void set(double *data) {
    location[0] = data[0];
    location[1] = data[1];
    location[2] = data[2];
    heading = data[3];
  }
  static CameraGeometry get(const double *data) {
    CameraGeometry out;
    out.location[0] = data[0];
    out.location[1] = data[1];
    out.location[2] = data[2];
    out.heading = data[3];
    return out;
  }

  void dump(std::basic_ostream<char> &o) const;
};

struct IntrinsicParams {
  static constexpr size_t kNumParams = 3;

  double mount_angle = 0.819433 / 180.0 * M_PI;  // 9.32615 / 180.0 * M_PI;
  double focal_length = 666.763;                 // 734.328;
  // This is a final rotation where the camera isn't straight.
  double barrel_mount = 2.72086 / 180.0 * M_PI;

  void set(double *data) {
    data[0] = mount_angle;
    data[1] = focal_length;
    data[2] = barrel_mount;
  }
  static IntrinsicParams get(const double *data) {
    IntrinsicParams out;
    out.mount_angle = data[0];
    out.focal_length = data[1];
    out.barrel_mount = data[2];
    return out;
  }
  void dump(std::basic_ostream<char> &o) const;
};

// Metadata about the calibration results (Should be good enough to reproduce).
struct DatasetInfo {
  int camera_id;
  // In meters from IMU start.
  std::array<double, 2> to_tape_measure_start;
  // In meters,
  std::array<double, 2> tape_measure_direction;
  // This will multiply tape_measure_direction and thus has no units.
  double beginning_tape_measure_reading;
  const char *filename_prefix;
  int num_images;

  void dump(std::basic_ostream<char> &o) const;
};

struct CameraCalibration {
  IntrinsicParams intrinsics;
  CameraGeometry geometry;
  DatasetInfo dataset;
};

const CameraCalibration *GetCamera(int camera_id);

void DumpCameraConstants(int camera_id, const CameraCalibration &value);

}  // namespace vision
}  // namespace y2019

#endif  // _Y2019_VISION_CONSTANTS_H_
