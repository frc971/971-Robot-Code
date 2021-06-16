package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_library(
    name = "control_loop_test",
    testonly = True,
    srcs = [
        "control_loop_test.cc",
    ],
    hdrs = [
        "control_loop_test.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//frc971/input:joystick_state_fbs",
        "//frc971/input:robot_state_fbs",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/time",
    ],
)

cc_library(
    name = "polytope",
    hdrs = [
        "polytope.h",
    ],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ] + select({
        "@platforms//os:linux": [
            "//aos/logging",
            "//third_party/cddlib",
            "@com_github_google_glog//:glog",
        ],
        "//conditions:default": [],
    }),
)

cc_test(
    name = "polytope_test",
    srcs = [
        "polytope_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":polytope",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

flatbuffer_cc_library(
    name = "control_loop_fbs",
    srcs = [
        "control_loops.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "control_loop",
    srcs = [
        "control_loop.cc",
        "control_loop-tmpl.h",
    ],
    hdrs = [
        "control_loop.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//frc971/input:joystick_state_fbs",
        "//frc971/input:robot_state_fbs",
        "//aos/time",
        "//aos/util:log_interval",
    ],
)

cc_library(
    name = "quaternion_utils",
    srcs = [
        "quaternion_utils.cc",
    ],
    hdrs = [
        "quaternion_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_glog//:glog",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "quarternion_utils_test",
    srcs = [
        "quaternion_utils_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":quaternion_utils",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "@com_github_google_glog//:glog",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
