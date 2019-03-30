#include "y2019/vision/constants.h"

namespace y2019 {
namespace vision {

static constexpr double kInchesToMeters = 0.0254;

CameraCalibration camera_1 = {
    {
        -1.01208 / 180.0 * M_PI, 342.679, 1.79649 / 180.0 * M_PI,
    },
    {
        {{-5.08996 * kInchesToMeters, 1.82468 * kInchesToMeters,
          33.2047 * kInchesToMeters}},
        -178.111 / 180.0 * M_PI,
    },
    {
        1,
        {{-12.5 * kInchesToMeters, -1 * kInchesToMeters}},
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
        -1.15844 / 180.0 * M_PI, 348.161, 1.16894 / 180.0 * M_PI,
    },
    {
        {{4.73183 * kInchesToMeters, 2.0984 * kInchesToMeters,
          33.2023 * kInchesToMeters}},
        -11.8598 / 180.0 * M_PI,
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
        -2.24098 / 180.0 * M_PI, 339.231, 1.15487 / 180.0 * M_PI,
    },
    {
        {{3.50224 * kInchesToMeters, 3.95441 * kInchesToMeters,
          33.3469 * kInchesToMeters}},
        18.6782 / 180.0 * M_PI,
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
        37.1859 / 180.0 * M_PI, 339.517, 0.0405714 / 180.0 * M_PI,
    },
    {
        {{3.57002 * kInchesToMeters, 5.26966 * kInchesToMeters,
          12.6807 * kInchesToMeters}},
        92.6787 / 180.0 * M_PI,
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
        35.4154 / 180.0 * M_PI, 337.471, 3.30546 / 180.0 * M_PI,
    },
    {
        {{4.25679 * kInchesToMeters, -2.93066 * kInchesToMeters,
          11.3228 * kInchesToMeters}},
        -93.219 / 180.0 * M_PI,
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
        -0.305107 / 180.0 * M_PI, 336.952, -0.0804389 / 180.0 * M_PI,
    },
    {
        {{-5.64467 * kInchesToMeters, 2.41348 * kInchesToMeters,
          33.1669 * kInchesToMeters}},
        181.598 / 180.0 * M_PI,
    },
    {
        10,
        {{-12.5 * kInchesToMeters, 0.0}},
        {{-1 * kInchesToMeters, 0.0}},
        11,
        "data/cam10_1/debug_viewer_jpeg_",
        69,
    }};

CameraCalibration camera_14 = {
    {
        0.108434 / 180.0 * M_PI, 338.756, 0.606249 / 180.0 * M_PI,
    },
    {
        {{5.90372 * kInchesToMeters, 2.08009 * kInchesToMeters,
          33.1342 * kInchesToMeters}},
        -12.4624 / 180.0 * M_PI,
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
        -0.855459 / 180.0 * M_PI, 348.799, 1.4559 / 180.0 * M_PI,
    },
    {
        {{3.15291 * kInchesToMeters, 4.16556 * kInchesToMeters,
          33.5924 * kInchesToMeters}},
        20.3884 / 180.0 * M_PI,
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
        34.8231 / 180.0 * M_PI, 338.051, 2.43035 / 180.0 * M_PI,
    },
    {
        {{3.17222 * kInchesToMeters, -2.49752 * kInchesToMeters,
          11.8333 * kInchesToMeters}},
        -92.1018 / 180.0 * M_PI,
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
        33.9761 / 180.0 * M_PI, 338.017, -2.32243 / 180.0 * M_PI,
    },
    {
        {{3.95182 * kInchesToMeters, 5.50479 * kInchesToMeters,
          12.3506 * kInchesToMeters}},
        96.4141 / 180.0 * M_PI,
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
