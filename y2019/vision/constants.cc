#include "y2019/vision/constants.h"

namespace y2019 {
namespace vision {

constexpr double kInchesToMeters = 0.0254;

CameraCalibration camera_4 = {
    {
        3.50309 / 180.0 * M_PI, 593.557, -0.0487739 / 180.0 * M_PI,
    },
    {
        {{5.56082 / kInchesToMeters, 4.70235 / kInchesToMeters,
          33.4998 / kInchesToMeters}},
        22.2155 * M_PI / 180.0,
    },
    {
        4,
        {{12.5 / kInchesToMeters, 12.0 / kInchesToMeters}},
        {{kInchesToMeters, 0.0}},
        26.0,
        "cam4_0/debug_viewer_jpeg_",
    }};

const CameraCalibration *GetCamera(int camera_id) {
  switch (camera_id) {
  case 4: return &camera_4;
  default: return nullptr;
  }
}

}  // namespace vision
}  // namespace y2019
