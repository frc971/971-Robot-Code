package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

filegroup(
    name = "binaries",
    srcs = [
        ":autonomous_action",
        ":superstructure_action",
    ],
)

flatbuffer_cc_library(
    name = "superstructure_action_fbs",
    srcs = [
        "superstructure_action.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "superstructure_action_lib",
    srcs = [
        "superstructure_actor.cc",
    ],
    hdrs = [
        "superstructure_actor.h",
    ],
    deps = [
        ":superstructure_action_fbs",
        "//aos/actions:action_lib",
        "//aos/events:event_loop",
        "//aos/logging",
        "//aos/util:phased_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//y2016/control_loops/superstructure:superstructure_goal_fbs",
        "//y2016/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_binary(
    name = "superstructure_action",
    srcs = [
        "superstructure_actor_main.cc",
    ],
    deps = [
        ":superstructure_action_fbs",
        ":superstructure_action_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_library(
    name = "autonomous_action_lib",
    srcs = [
        "autonomous_actor.cc",
    ],
    hdrs = [
        "autonomous_actor.h",
    ],
    deps = [
        ":vision_align_action_lib",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/util:phased_loop",
        "//frc971/autonomous:base_autonomous_actor",
        "//y2016/control_loops/drivetrain:drivetrain_base",
        "//y2016/control_loops/shooter:shooter_goal_fbs",
        "//y2016/control_loops/shooter:shooter_status_fbs",
        "//y2016/control_loops/superstructure:superstructure_goal_fbs",
        "//y2016/control_loops/superstructure:superstructure_status_fbs",
        "//y2016/queues:ball_detector_fbs",
        "//y2016/vision:vision_fbs",
    ],
)

cc_binary(
    name = "autonomous_action",
    srcs = [
        "autonomous_actor_main.cc",
    ],
    deps = [
        ":autonomous_action_lib",
        "//aos:init",
    ],
)

flatbuffer_cc_library(
    name = "vision_align_action_fbs",
    srcs = [
        "vision_align_action.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "vision_align_action_lib",
    srcs = [
        "vision_align_actor.cc",
    ],
    hdrs = [
        "vision_align_actor.h",
    ],
    deps = [
        ":vision_align_action_fbs",
        "//aos:math",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//aos/time",
        "//aos/util:phased_loop",
        "//aos/util:trapezoid_profile",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_goal_fbs",
        "//y2016:constants",
        "//y2016/control_loops/drivetrain:drivetrain_base",
        "//y2016/vision:vision_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "vision_align_action",
    srcs = [
        "vision_align_actor_main.cc",
    ],
    deps = [
        ":vision_align_action_fbs",
        ":vision_align_action_lib",
        "//aos:init",
    ],
)
