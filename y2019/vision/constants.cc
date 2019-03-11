#include "y2019/vision/constants.h"

namespace y2019 {
namespace vision {

static constexpr double kInchesToMeters = 0.0254;

CameraCalibration camera_1 = {
    {
        -0.874694 / 180.0 * M_PI, 338.619, 2.14651 / 180.0 * M_PI,
    },
    {
        {{-5.44063 * kInchesToMeters, 2.83405 * kInchesToMeters,
          33.0386 * kInchesToMeters}},
        181.723 / 180.0 * M_PI,
    },
    {
        1,
        {{-12.5 * kInchesToMeters, 0.0}},
        {{-1 * kInchesToMeters, 0.0}},
        16,
        "/home/alex/cam1/debug_viewer_jpeg_",
        45,
    }};

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

CameraCalibration camera_6 = {
    {
        -1.11982 / 180.0 * M_PI, 347.32, 1.04799 / 180.0 * M_PI,
    },
    {
        {{4.84356 * kInchesToMeters, 2.12355 * kInchesToMeters,
          33.1812 * kInchesToMeters}},
        -11.888 / 180.0 * M_PI,
    },
    {
        6,
        {{12.5 * kInchesToMeters, 0.0}},
        {{1 * kInchesToMeters, 0.0}},
        11,
        "data/cam6_0/debug_viewer_jpeg_",
        75,
    }};

CameraCalibration camera_7 = {
    {
        -2.21509 / 180.0 * M_PI, 328.2, 0.922133 / 180.0 * M_PI,
    },
    {
        {{4.7585 * kInchesToMeters, 3.99542 * kInchesToMeters,
          33.1932 * kInchesToMeters}},
        19.2585 / 180.0 * M_PI,
    },
    {
        7,
        {{12.5 * kInchesToMeters, 0.0}},
        {{1 * kInchesToMeters, 0.0}},
        21,
        "data/cam7_0/debug_viewer_jpeg_",
        65,
    }};

CameraCalibration camera_8 = {
    {
        37.1314 / 180.0 * M_PI, 340.763, -0.136505 / 180.0 * M_PI,
    },
    {
        {{3.61159 * kInchesToMeters, 5.16559 * kInchesToMeters,
          12.6279 * kInchesToMeters}},
        92.78 / 180.0 * M_PI,
    },
    {
        8,
        {{6.5 * kInchesToMeters, -11 * kInchesToMeters}},
        {{0.0, 1 * kInchesToMeters}},
        25,
        "data/cam8_0/debug_viewer_jpeg_",
        61,
    }};

CameraCalibration camera_9 = {
    {
        35.4057 / 180.0 * M_PI, 337.052, 3.57896 / 180.0 * M_PI,
    },
    {
        {{4.28266 * kInchesToMeters, -3.04674 * kInchesToMeters,
          11.4219 * kInchesToMeters}},
        -93.3496 / 180.0 * M_PI,
    },
    {
        9,
        {{-6.5 * kInchesToMeters, 11 * kInchesToMeters}},
        {{0.0, -1 * kInchesToMeters}},
        30,
        "data/cam9_0/debug_viewer_jpeg_",
        56,
    }};

CameraCalibration camera_10 = {
    {
        -0.263033 / 180.0 * M_PI, 347.579, 0.213753 / 180.0 * M_PI,
    },
    {
        {{-4.71537 * kInchesToMeters, 2.87784 * kInchesToMeters,
          33.3979 * kInchesToMeters}},
        182.17 / 180.0 * M_PI,
    },
    {
        10,
        {{-12.5 * kInchesToMeters, 0.0}},
        {{-1 * kInchesToMeters, 0.0}},
        11,
        "data/cam10_0/debug_viewer_jpeg_",
        75,
    }};

CameraCalibration camera_14 = {
    {
        -0.0729684 / 180.0 * M_PI, 343.569, 0.685893 / 180.0 * M_PI,
    },
    {
        {{5.53867 * kInchesToMeters, 2.08897 * kInchesToMeters,
          33.1766 * kInchesToMeters}},
        -12.4121 / 180.0 * M_PI,
    },
    {
        14,
        {{12.5 * kInchesToMeters, 0.0}},
        {{1 * kInchesToMeters, 0.0}},
        16,
        "/home/alex/cam14/debug_viewer_jpeg_",
        71,
    }};

CameraCalibration camera_15 = {
    {
        -0.715538 / 180.0 * M_PI, 336.509, 1.54169 / 180.0 * M_PI,
    },
    {
        {{4.57935 * kInchesToMeters, 4.16624 * kInchesToMeters,
          33.433 * kInchesToMeters}},
        20.9856 / 180.0 * M_PI,
    },
    {
        15,
        {{12.5 * kInchesToMeters, 0.0}},
        {{1 * kInchesToMeters, 0.0}},
        25,
        "/home/alex/cam15/debug_viewer_jpeg_",
        62,
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

CameraCalibration camera_17 = {
    {
        34.7631 / 180.0 * M_PI, 337.946, 2.48943 / 180.0 * M_PI,
    },
    {
        {{3.15808 * kInchesToMeters, -2.5734 * kInchesToMeters,
          11.8507 * kInchesToMeters}},
        -92.1953 / 180.0 * M_PI,
    },
    {
        17,
        {{-6.5 * kInchesToMeters, 11 * kInchesToMeters}},
        {{0.0, -1 * kInchesToMeters}},
        29,
        "/home/alex/cam17/debug_viewer_jpeg_",
        58,
    }};

CameraCalibration camera_18 = {
    {
        33.9292 / 180.0 * M_PI, 338.44, -1.71889 / 180.0 * M_PI,
    },
    {
        {{3.82945 * kInchesToMeters, 5.51444 * kInchesToMeters,
          12.3803 * kInchesToMeters}},
        96.0112 / 180.0 * M_PI,
    },
    {
        18,
        {{6.5 * kInchesToMeters, -11 * kInchesToMeters}},
        {{0.0, 1 * kInchesToMeters}},
        27,
        "/home/alex/cam18/debug_viewer_jpeg_",
        60,
    }};

CameraCalibration camera_19 = {
    {
        -0.341036 / 180.0 * M_PI, 324.626, 1.2545 / 180.0 * M_PI,
    },
    {
        {{-6.93309 * kInchesToMeters, 2.64735 * kInchesToMeters,
          32.8758 * kInchesToMeters}},
        -177.419 / 180.0 * M_PI,
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
    case 1:
      return &camera_1;
    case 4:
      return &camera_4;
    case 5:
      return &camera_5;
    case 6:
      return &camera_6;
    case 7:
      return &camera_7;
    case 8:
      return &camera_8;
    case 9:
      return &camera_9;
    case 10:
      return &camera_10;
    case 14:
      return &camera_14;
    case 15:
      return &camera_15;
    case 16:
      return &camera_16;
    case 17:
      return &camera_17;
    case 18:
      return &camera_18;
    case 19:
      return &camera_19;
    default:
      return nullptr;
  }
}

}  // namespace vision
}  // namespace y2019
