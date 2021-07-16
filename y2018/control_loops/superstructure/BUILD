package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "superstructure_position_fbs",
    srcs = [
        "superstructure_position.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "superstructure_status_fbs",
    srcs = [
        "superstructure_status.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

flatbuffer_cc_library(
    name = "superstructure_output_fbs",
    srcs = [
        "superstructure_output.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "superstructure_lib",
    srcs = [
        "superstructure.cc",
    ],
    hdrs = [
        "superstructure.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos/events:event_loop",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_output_fbs",
        "//y2018:constants",
        "//y2018:status_light_fbs",
        "//y2018/control_loops/superstructure/arm",
        "//y2018/control_loops/superstructure/intake",
        "//y2018/vision:vision_fbs",
    ],
)

cc_test(
    name = "superstructure_lib_test",
    timeout = "long",
    srcs = [
        "superstructure_lib_test.cc",
    ],
    data = ["//y2018:config"],
    shard_count = 5,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_lib",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:math",
        "//aos/testing:googletest",
        "//aos/time",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//y2018/control_loops/superstructure/intake:intake_plants",
    ],
)

cc_binary(
    name = "superstructure",
    srcs = [
        "superstructure_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_lib",
        "//aos:init",
    ],
)

cc_library(
    name = "debouncer",
    srcs = [
        "debouncer.cc",
    ],
    hdrs = [
        "debouncer.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "debouncer_test",
    srcs = [
        "debouncer_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":debouncer",
        "//aos/testing:googletest",
    ],
)
