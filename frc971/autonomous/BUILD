package(default_visibility = ["//visibility:public"])

load("//aos:config.bzl", "aos_config")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "auto_fbs",
    srcs = ["auto.fbs"],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "auto_mode_fbs",
    srcs = ["auto_mode.fbs"],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "base_autonomous_actor",
    srcs = [
        "base_autonomous_actor.cc",
    ],
    hdrs = [
        "base_autonomous_actor.h",
    ],
    deps = [
        ":auto_fbs",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/util:phased_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//frc971/control_loops/drivetrain:drivetrain_goal_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/control_loops/drivetrain:localizer_fbs",
        "//y2019/control_loops/drivetrain:target_selector_fbs",
    ],
)

aos_config(
    name = "config",
    src = "autonomous_config.json",
    flatbuffers = [
        "//aos/actions:actions_fbs",
        ":auto_fbs",
        ":auto_mode_fbs",
    ],
    visibility = ["//visibility:public"],
)
