load("//tools:platforms.bzl", "platforms")
load("//tools/build_rules:select.bzl", "cpu_select")

# Incompatible library in case one isn't available for a specific architecture.
cc_library(
    name = "unavailable",
    target_compatible_with = ["@platforms//:incompatible"],
)

cc_library(
    name = "wpilib",
    linkstatic = True,
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//frc971/wpilib/ahal"],
)

cc_library(
    name = "wpilib_hal",
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//third_party/allwpilib:hal"],
)

cc_library(
    name = "phoenix",
    linkopts = [
        "-Wl,-rpath",
        "-Wl,.",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "@ctre_phoenix_api_cpp_athena//:api-cpp",
        "@ctre_phoenix_api_cpp_headers//:api-cpp",
        "@ctre_phoenix_cci_athena//:cci",
        "@ctre_phoenix_cci_headers//:cci",
    ],
)

cc_library(
    name = "opencv",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@opencv_k8//:opencv"],
        "//tools:cpu_armhf": ["@opencv_armhf//:opencv"],
        "//tools:cpu_arm64": ["@opencv_arm64//:opencv"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@gstreamer_k8//:gstreamer"],
        "//tools:cpu_armhf": ["@gstreamer_armhf//:gstreamer"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "halide",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:halide"],
        "//tools:cpu_armhf": ["@halide_armhf//:halide"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "halide_gengen",
    visibility = ["//visibility:public"],
    # It's the same file in either version, but we'll pick the native version
    # to minimize the chances of needing to download the other version unnecessarily.
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:gengen"],
        "//tools:cpu_armhf": ["@halide_armhf//:gengen"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "halide_runtime",
    visibility = ["//visibility:public"],
    # It's the same file in either version, but we'll pick the native version
    # to minimize the chances of needing to download the other version unnecessarily.
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:runtime"],
        "//tools:cpu_armhf": ["@halide_armhf//:runtime"],
        "//tools:cpu_arm64": ["@halide_arm64//:runtime"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "lzma",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@lzma_amd64//:lib"],
        "//tools:cpu_arm64": ["@lzma_arm64//:lib"],
        "//conditions:default": [":unavailable"],
    }),
)
