#include "y2019/vision/constants.h"

namespace y2019 {
namespace vision {

static constexpr double kInchesToMeters = 0.0254;

CameraCalibration camera_4 = {
    {
     3.73623 / 180.0 * M_PI, 588.1, 0.269291 / 180.0 * M_PI,
    },
    {
     {{6.02674 * kInchesToMeters, 4.57805 * kInchesToMeters,
       33.3849 * kInchesToMeters}},
     22.4535 / 180.0 * M_PI,
    },
    {
     4,
     {{12.5 * kInchesToMeters, 12 * kInchesToMeters}},
     {{1 * kInchesToMeters, 0.0}},
     26,
     "cam4_0/debug_viewer_jpeg_",
     52,
    }};

CameraCalibration camera_5 = {
    {
     1.00774 / 180.0 * M_PI, 658.554, 2.43864 / 180.0 * M_PI,
    },
    {
     {{5.51248 * kInchesToMeters, 2.04087 * kInchesToMeters,
       33.2555 * kInchesToMeters}},
     -13.1396 / 180.0 * M_PI,
    },
    {
     5,
     {{12.5 * kInchesToMeters, 0.5 * kInchesToMeters}},
     {{1 * kInchesToMeters, 0.0}},
     26,
     "cam5_0/debug_viewer_jpeg_",
     59,
    }};

CameraCalibration camera_16 = {
    {
     -1.30906 / 180.0 * M_PI, 347.372, 2.18486 / 180.0 * M_PI,
    },
    {
     {{4.98126 * kInchesToMeters, 1.96988 * kInchesToMeters,
       33.4276 * kInchesToMeters}},
     -12.2377 / 180.0 * M_PI,
    },
    {
     16,
     {{12.5 * kInchesToMeters, 0.5 * kInchesToMeters}},
     {{1 * kInchesToMeters, 0.0}},
     16,
     "cam16/debug_viewer_jpeg_",
     55,
    }};

// Note: x/y should be negated and heading should be offset by 180 deg to
// account for this being calibrated on the rear of the robot.
CameraCalibration camera_19 = {
    {
     -0.341036 / 180.0 * M_PI, 324.626, 1.2545 / 180.0 * M_PI,
    },
    {
     {{6.93309 * kInchesToMeters, -2.64735 * kInchesToMeters,
       32.8758 * kInchesToMeters}},
     2.58102 / 180.0 * M_PI,
    },
    {
     19,
     {{12.5 * kInchesToMeters, 0.5 * kInchesToMeters}},
     {{1 * kInchesToMeters, 0.0}},
     16,
     "cam19/debug_viewer_jpeg_",
     68,
    }};

const CameraCalibration *GetCamera(int camera_id) {
  switch (camera_id) {
  case 4: return &camera_4;
  case 5: return &camera_5;
  case 16: return &camera_16;
  case 19: return &camera_19;
  default: return nullptr;
  }
}

}  // namespace vision
}  // namespace y2019
