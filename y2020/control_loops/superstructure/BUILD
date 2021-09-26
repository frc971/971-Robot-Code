package(default_visibility = ["//visibility:public"])

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
load("@npm_bazel_typescript//:defs.bzl", "ts_library")

flatbuffer_cc_library(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
        "//frc971/control_loops:profiled_subsystem_fbs_includes",
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

flatbuffer_ts_library(
    name = "superstructure_status_ts_fbs",
    srcs = [
        "superstructure_status.fbs",
    ],
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
        "//frc971/control_loops:profiled_subsystem_fbs_includes",
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
        "//frc971/control_loops:profiled_subsystem_fbs_includes",
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
        "//frc971/control_loops:profiled_subsystem_fbs_includes",
    ],
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
        ":climber",
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos/events:event_loop",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//y2020:constants",
        "//y2020/control_loops/superstructure/shooter",
        "//y2020/control_loops/superstructure/turret:aiming",
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
        "//aos/events:shm_event_loop",
    ],
)

cc_test(
    name = "superstructure_lib_test",
    srcs = [
        "superstructure_lib_test.cc",
    ],
    data = [
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_lib",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:math",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "//aos/time",
        "//frc971/control_loops:capped_test_plant",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//y2020/control_loops/superstructure/hood:hood_plants",
        "//y2020/control_loops/superstructure/intake:intake_plants",
        "//y2020/control_loops/superstructure/shooter:shooter_plants",
    ],
)

cc_library(
    name = "climber",
    srcs = [
        "climber.cc",
    ],
    hdrs = [
        "climber.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
    ],
)

ts_library(
    name = "turret_plotter",
    srcs = ["turret_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration_ts_fbs",
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:plotter",
        "//aos/network/www:proxy",
        "//aos/network/www:reflection_ts",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_library(
    name = "finisher_plotter",
    srcs = ["finisher_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

ts_library(
    name = "accelerator_plotter",
    srcs = ["accelerator_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

ts_library(
    name = "hood_plotter",
    srcs = ["hood_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)
