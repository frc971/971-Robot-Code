load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "claw_goal_fbs",
    srcs = [
        "claw_goal.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "claw_position_fbs",
    srcs = [
        "claw_position.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:control_loops_fbs",
    ],
)

static_flatbuffer(
    name = "claw_output_fbs",
    srcs = [
        "claw_output.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "claw_status_fbs",
    srcs = [
        "claw_status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

genrule(
    name = "genrule_claw",
    outs = [
        "claw_motor_plant.h",
        "claw_motor_plant.cc",
    ],
    cmd = "$(location //y2014/control_loops/python:claw) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2014/control_loops/python:claw",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "claw_lib",
    srcs = [
        "claw.cc",
        "claw_motor_plant.cc",
    ],
    hdrs = [
        "claw.h",
        "claw_motor_plant.h",
    ],
    linkopts = [
        "-lm",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":claw_goal_fbs",
        ":claw_output_fbs",
        ":claw_position_fbs",
        ":claw_status_fbs",
        "//aos:math",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:hall_effect_tracker",
        "//frc971/control_loops:polytope",
        "//frc971/control_loops:state_feedback_loop",
        "//y2014:constants",
    ],
)

cc_test(
    name = "claw_lib_test",
    srcs = [
        "claw_lib_test.cc",
    ],
    data = ["//y2014:aos_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":claw_goal_fbs",
        ":claw_lib",
        ":claw_output_fbs",
        ":claw_position_fbs",
        ":claw_status_fbs",
        "//aos/testing:googletest",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops:team_number_test_environment",
    ],
)

cc_binary(
    name = "claw",
    srcs = [
        "claw_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":claw_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
