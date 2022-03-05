load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
load("@npm//@bazel/typescript:index.bzl", "ts_library")

package(default_visibility = ["//visibility:public"])

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
)

flatbuffer_cc_library(
    name = "superstructure_output_fbs",
    srcs = [
        "superstructure_output.fbs",
    ],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "superstructure_can_position_fbs",
    srcs = [
        "superstructure_can_position.fbs",
    ],
    gen_reflections = 1,
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
    name = "superstructure_position_fbs",
    srcs = [
        "superstructure_position.fbs",
    ],
    gen_reflections = 1,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
        "//frc971/control_loops:profiled_subsystem_fbs_includes",
    ],
)

cc_library(
    name = "superstructure_lib",
    srcs = [
        "superstructure.cc",
    ],
    hdrs = [
        "superstructure.h",
    ],
    deps = [
        ":collision_avoidance_lib",
        ":superstructure_can_position_fbs",
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos/events:event_loop",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//y2022:constants",
        "//y2022/control_loops/superstructure/catapult",
    ],
)

cc_binary(
    name = "superstructure",
    srcs = [
        "superstructure_main.cc",
    ],
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
        "//y2022:aos_config",
    ],
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
    ],
)

cc_library(
    name = "collision_avoidance_lib",
    srcs = ["collision_avoidance.cc"],
    hdrs = ["collision_avoidance.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/functional:bind_front",
    ],
)

cc_test(
    name = "collision_avoidance_test",
    srcs = ["collision_avoidance_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":collision_avoidance_lib",
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//aos:flatbuffers",
        "//aos:math",
        "//aos/testing:googletest",
    ],
)

ts_library(
    name = "superstructure_plotter",
    srcs = ["superstructure_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

ts_library(
    name = "catapult_plotter",
    srcs = ["catapult_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

ts_library(
    name = "intake_plotter",
    srcs = ["intake_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

ts_library(
    name = "turret_plotter",
    srcs = ["turret_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

ts_library(
    name = "climber_plotter",
    srcs = ["climber_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)
