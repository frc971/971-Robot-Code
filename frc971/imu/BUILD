load("//third_party/pico-sdk:hex.bzl", "pioasm", "uf2_from_elf")

# TODO(austin): We should be able to either push the linker script into the crosstool, or wrap this in a macro.
cc_binary(
    name = "blink.elf",
    srcs = [
        "blink.c",
        "//third_party/pico-sdk/src/rp2_common/boot_stage2:bs2_default_padded_checksummed.S",
    ],
    additional_linker_inputs = [
        "//third_party/pico-sdk:src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    linkopts = [
        "-Wl,--script=third_party/pico-sdk/src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m0plus",
    ],
    deps = ["//third_party/pico-sdk"],
)

uf2_from_elf(
    name = "blink",
)

pioasm(
    name = "quadrature_encoder",
    src = "quadrature_encoder.pio",
    out = "quadrature_encoder.pio.h",
)

# TODO(austin): We should be able to either push the linker script into the crosstool, or wrap this in a macro.
cc_binary(
    name = "ADIS16505.elf",
    srcs = [
        "ADIS16505.cc",
        "quadrature_encoder.pio.h",
        "//third_party/pico-sdk/src/rp2_common/boot_stage2:bs2_default_padded_checksummed.S",
    ],
    additional_linker_inputs = [
        "//third_party/pico-sdk:src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    copts = [
        "-Wno-unused-parameter",
        "-Wno-unused-variable",
        "-Wno-double-promotion",
        "-Wno-unused-function",
    ],
    includes = ["."],
    linkopts = [
        "-Wl,--script=third_party/pico-sdk/src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m0plus",
    ],
    deps = ["//third_party/pico-sdk"],
)

uf2_from_elf(
    name = "ADIS16505",
)

cc_library(
    name = "imu_calibrator",
    srcs = ["imu_calibrator.cc"],
    hdrs = [
        "imu_calibrator.h",
        "imu_calibrator-tmpl.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/time",
        "//frc971/math:interpolate",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "imu_calibrator_solver",
    srcs = [
        "imu_calibrator_solver.cc",
    ],
    hdrs = [
        "imu_calibrator_solver.h",
    ],
    deps = [
        ":imu_calibrator",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "imu_calibrator_test",
    srcs = ["imu_calibrator_test.cc"],
    shard_count = 3,
    deps = [
        ":imu_calibrator",
        ":imu_calibrator_solver",
        "//aos/testing:googletest",
    ],
)
