#include "y2019/vision/constants.h"

#include <fstream>
#include <sstream>

namespace y2019 {
namespace vision {

namespace {
// 64 should be enough for any mortal.
constexpr int kMaxNumCameras = 64;
constexpr double kInchesToMeters = 0.0254;
}  // namespace

static std::string fmt_rad(double v) {
  std::stringstream ss;
  if (v == 0.0) {
    ss << "0.0";
  } else {
    ss << v * 180.0 / M_PI << " / 180.0 * M_PI";
  }
  return ss.str();
}

static std::string fmt_meters(double v) {
  if (v == 0.0) return "0.0";
  if (v == 1.0) return "kInchesToMeters";
  std::stringstream ss;
  ss << v / kInchesToMeters << " * kInchesToMeters";
  return ss.str();
}

void IntrinsicParams::Dump(std::basic_ostream<char> *o) const {
  *o << "    {\n        " << fmt_rad(mount_angle) << ", " << focal_length;
  *o << ", " << fmt_rad(barrel_mount) << ",\n    },\n";
}

void CameraGeometry::Dump(std::basic_ostream<char> *o) const {
  *o << "    {\n        {{" << fmt_meters(location[0]) << ", "
     << fmt_meters(location[1]) << ",\n          " << fmt_meters(location[2])
     << "}},\n        " << fmt_rad(heading) << ",\n    },\n    ";
}

void DatasetInfo::Dump(std::basic_ostream<char> *o) const {
  *o << "{\n        " << camera_id << ",\n        "
     << "{{" << fmt_meters(to_tape_measure_start[0]) << ", "
     << fmt_meters(to_tape_measure_start[1]) << "}},\n        "
     << "{{" << fmt_meters(tape_measure_direction[0]) << ", "
     << fmt_meters(tape_measure_direction[1]) << "}},\n        "
     << beginning_tape_measure_reading << ",\n        "
     << "\"" << filename_prefix << "\",\n        " << num_images << ",\n    }";
}

void DumpCameraConstants(const char *fname, int camera_id,
                         const CameraCalibration &value) {
  std::ofstream o(fname);
  o << R"(#include "y2019/vision/constants.h"

namespace y2019 {
namespace vision {

static constexpr double kInchesToMeters = 0.0254;
)";

  // Go through all the cameras and either use the existing compiled-in
  // calibration data or the new data which was passed in.
  for (int i = 0; i < kMaxNumCameras; ++i) {
    auto *params = (i == camera_id) ? &value : GetCamera(i);
    if (params) {
      o << "\nCameraCalibration camera_" << i << " = {\n";
      params->intrinsics.Dump(&o);
      params->geometry.Dump(&o);
      params->dataset.Dump(&o);
      o << "};\n";
    }
  }

  o << R"(
const CameraCalibration *GetCamera(int camera_id) {
  switch (camera_id) {
)";
  for (int i = 0; i < kMaxNumCameras; ++i) {
    if (i == camera_id || GetCamera(i) != nullptr) {
      o << "    case " << i << ":\n      return &camera_" << i << ";\n";
    }
  }
  o << R"(    default:
      return nullptr;
  }
}

}  // namespace vision
}  // namespace y2019
)";
  o.close();
}

}  // namespace vision
}  // namespace y2019
