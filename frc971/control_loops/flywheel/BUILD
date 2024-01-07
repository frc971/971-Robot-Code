load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

flatbuffer_cc_library(
    name = "flywheel_controller_status_fbs",
    srcs = [
        "flywheel_controller_status.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "flywheel_controller_status_ts_fbs",
    srcs = [
        "flywheel_controller_status.fbs",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "flywheel_test_plant",
    hdrs = [
        "flywheel_test_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":flywheel_controller",
    ],
)

cc_library(
    name = "flywheel_controller",
    srcs = [
        "flywheel_controller.cc",
    ],
    hdrs = [
        "flywheel_controller.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":flywheel_controller_status_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:profiled_subsystem",
    ],
)
