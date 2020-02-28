load("@//tools/build_rules:select.bzl", "cpu_select")

cc_library(
    name = "wpilib",
    linkstatic = True,
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//frc971/wpilib/ahal"],
)

cc_library(
    name = "wpilib_hal",
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//third_party/allwpilib:hal"],
)

cc_library(
    name = "phoenix",
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "@ctre_phoenix_api_cpp_athena//:api-cpp",
        "@ctre_phoenix_api_cpp_headers//:api-cpp",
        "@ctre_phoenix_cci_athena//:cci",
        "@ctre_phoenix_cci_headers//:cci",
        "@ctre_phoenix_diagnostics_athena//:diagnostics",
        "@ctre_phoenix_diagnostics_headers//:diagnostics",
    ],
)

cc_library(
    name = "opencv",
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@opencv_k8//:opencv"],
        "//tools:cpu_armhf": ["@opencv_armhf//:opencv"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "halide",
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:halide"],
        "//tools:cpu_armhf": ["@halide_armhf//:halide"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "halide_gengen",
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    # It's the same file in either version, but we'll pick the native version
    # to minimize the chances of needing to download the other version unnecessarily.
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:gengen"],
        "//tools:cpu_armhf": ["@halide_armhf//:gengen"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "halide_runtime",
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    # It's the same file in either version, but we'll pick the native version
    # to minimize the chances of needing to download the other version unnecessarily.
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:runtime"],
        "//tools:cpu_armhf": ["@halide_armhf//:runtime"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "webrtc",
    visibility = ["//visibility:public"],
    deps = cpu_select({
        "amd64": ["@webrtc_x64//:webrtc"],
        "armhf": ["@webrtc_arm//:webrtc"],
        "cortex-m": ["@webrtc_arm//:webrtc"],
        "roborio": ["@webrtc_rio//:webrtc"],
    }),
)
