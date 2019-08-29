package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "control_loop_test",
    testonly = True,
    srcs = [
        "control_loop_test.cc",
    ],
    hdrs = [
        "control_loop_test.h",
    ],
    deps = [
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/robot_state:joystick_state_fbs",
        "//aos/robot_state:robot_state_fbs",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/time",
    ],
)

cc_library(
    name = "polytope_uc",
    hdrs = [
        "polytope.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "polytope",
    hdrs = [
        "polytope.h",
    ],
    deps = [
        "//aos/logging",
        "//third_party/cddlib",
        "@com_github_google_glog//:glog",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "polytope_test",
    srcs = [
        "polytope_test.cc",
    ],
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
    deps = [
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/robot_state:joystick_state_fbs",
        "//aos/robot_state:robot_state_fbs",
        "//aos/time",
        "//aos/util:log_interval",
    ],
)
