
cc_library(
  name = "VL53L1X",
  srcs = [
    "core/VL53L1X_api.c",
    "core/VL53L1X_calibration.c",
    "platform/vl53l1_platform.c",
  ],
  hdrs = [
    "core/VL53L1X_api.h",
    "core/VL53L1X_calibration.h",
    "platform/vl53l1_platform.h",
    "platform/vl53l1_types.h",
  ],
  target_compatible_with = [
    "@platforms//os:none",
    "@//tools/platforms/hardware:cortex_m0plus",
  ],
  copts = [
    "-Wno-unused-parameter",
  ],
  visibility = ["//visibility:public"],
  deps = [
    "@//third_party/pico-sdk",
  ],
)
