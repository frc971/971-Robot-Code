load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

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
        ":collision_avoidance",
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        ":vacuum",
        "//aos/events:event_loop",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/zeroing:absolute_encoder",
        "//frc971/zeroing:pot_and_absolute_encoder",
        "//y2019:constants",
        "//y2019:status_light_fbs",
    ],
)

cc_test(
    name = "superstructure_lib_test",
    srcs = [
        "superstructure_lib_test.cc",
    ],
    data = [
        "//y2019:aos_config",
    ],
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
        "//frc971/control_loops:capped_test_plant",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//y2019:status_light_fbs",
        "//y2019/control_loops/superstructure/intake:intake_plants",
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

cc_library(
    name = "collision_avoidance",
    srcs = [
        "collision_avoidance.cc",
    ],
    hdrs = [
        "collision_avoidance.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//frc971:constants",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
    ],
)

cc_library(
    name = "vacuum",
    srcs = [
        "vacuum.cc",
    ],
    hdrs = [
        "vacuum.h",
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

cc_test(
    name = "collision_avoidance_tests",
    srcs = [
        "collision_avoidance_tests.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":collision_avoidance",
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//aos:math",
        "//aos/testing:googletest",
    ],
)
