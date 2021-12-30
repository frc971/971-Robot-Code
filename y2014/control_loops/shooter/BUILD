load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "shooter_goal_fbs",
    srcs = [
        "shooter_goal.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "shooter_position_fbs",
    srcs = [
        "shooter_position.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "shooter_output_fbs",
    srcs = [
        "shooter_output.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "shooter_status_fbs",
    srcs = [
        "shooter_status.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

genrule(
    name = "genrule_shooter",
    outs = [
        "shooter_motor_plant.cc",
        "shooter_motor_plant.h",
        "unaugmented_shooter_motor_plant.cc",
        "unaugmented_shooter_motor_plant.h",
    ],
    cmd = "$(location //y2014/control_loops/python:shooter) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2014/control_loops/python:shooter",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "shooter_lib",
    srcs = [
        "shooter.cc",
        "shooter_motor_plant.cc",
        "unaugmented_shooter_motor_plant.cc",
    ],
    hdrs = [
        "shooter.h",
        "shooter_motor_plant.h",
        "unaugmented_shooter_motor_plant.h",
    ],
    linkopts = [
        "-lm",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":shooter_goal_fbs",
        ":shooter_output_fbs",
        ":shooter_position_fbs",
        ":shooter_status_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:state_feedback_loop",
        "//y2014:constants",
    ],
)

cc_test(
    name = "shooter_lib_test",
    srcs = [
        "shooter_lib_test.cc",
    ],
    data = ["//y2014:config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":shooter_goal_fbs",
        ":shooter_lib",
        ":shooter_output_fbs",
        ":shooter_position_fbs",
        ":shooter_status_fbs",
        "//aos/testing:googletest",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_binary(
    name = "shooter",
    srcs = [
        "shooter_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":shooter_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
