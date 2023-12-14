load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "shooter_goal_fbs",
    srcs = [
        "shooter_goal.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "shooter_position_fbs",
    srcs = [
        "shooter_position.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:control_loops_fbs",
    ],
)

static_flatbuffer(
    name = "shooter_output_fbs",
    srcs = [
        "shooter_output.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "shooter_status_fbs",
    srcs = [
        "shooter_status.fbs",
    ],
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
    data = ["//y2014:aos_config"],
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
