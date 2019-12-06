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
        ":collision_avoidance",
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        ":vacuum",
        "//aos/controls:control_loop",
        "//aos/events:event_loop",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
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
        "//y2019:config.json",
    ],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_lib",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:math",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//aos/time",
        "//frc971/control_loops:capped_test_plant",
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
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//aos/controls:control_loop",
        "//frc971:constants",
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
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        "//aos/controls:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
    ],
)

cc_test(
    name = "collision_avoidance_tests",
    srcs = [
        "collision_avoidance_tests.cc",
    ],
    deps = [
        ":collision_avoidance",
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//aos:math",
        "//aos/testing:googletest",
    ],
)
