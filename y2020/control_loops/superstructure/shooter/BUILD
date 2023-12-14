load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

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
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:profiled_subsystem",
        "//frc971/control_loops/flywheel:flywheel_controller",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_output_fbs",
        "//y2020/control_loops/superstructure:superstructure_position_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
        "//y2020/control_loops/superstructure/accelerator:accelerator_plants",
        "//y2020/control_loops/superstructure/finisher:finisher_plants",
    ],
)

static_flatbuffer(
    name = "shooter_tuning_readings_fbs",
    srcs = [
        "shooter_tuning_readings.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "shooter_tuning_params_fbs",
    srcs = [
        "shooter_tuning_params.fbs",
    ],
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
