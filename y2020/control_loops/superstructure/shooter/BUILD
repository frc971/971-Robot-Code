package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_library(
    name = "shooter_plants",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "shooter",
    srcs = [
        "shooter.cc",
    ],
    hdrs = [
        "shooter.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":flywheel_controller",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:profiled_subsystem",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_output_fbs",
        "//y2020/control_loops/superstructure:superstructure_position_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
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
    deps = [
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:profiled_subsystem",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
        "//y2020/control_loops/superstructure/accelerator:accelerator_plants",
        "//y2020/control_loops/superstructure/finisher:finisher_plants",
    ],
)

flatbuffer_cc_library(
    name = "shooter_tuning_readings_fbs",
    srcs = [
        "shooter_tuning_readings.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "shooter_tuning_params_fbs",
    srcs = [
        "shooter_tuning_params.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "shooter_tuning_params_setter",
    srcs = ["shooter_tuning_params_setter.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":shooter_tuning_params_fbs",
        ":shooter_tuning_readings_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)
