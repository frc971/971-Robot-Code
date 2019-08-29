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
    name = "superstructure_output_fbs",
    srcs = [
        "superstructure_output.fbs",
    ],
    gen_reflections = 1,
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
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        ":vision_distance_average",
        "//aos/controls:control_loop",
        "//aos/events:event_loop",
        "//y2017:constants",
        "//y2017/control_loops/superstructure/column",
        "//y2017/control_loops/superstructure/hood",
        "//y2017/control_loops/superstructure/intake",
        "//y2017/control_loops/superstructure/shooter",
    ],
)

cc_test(
    name = "superstructure_lib_test",
    srcs = [
        "superstructure_lib_test.cc",
    ],
    data = ["//y2017:config.json"],
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
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//y2017/control_loops/superstructure/column:column_plants",
        "//y2017/control_loops/superstructure/hood:hood_plants",
        "//y2017/control_loops/superstructure/intake:intake_plants",
        "//y2017/control_loops/superstructure/shooter:shooter_plants",
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
    name = "vision_time_adjuster",
    srcs = [
        "vision_time_adjuster.cc",
    ],
    hdrs = [
        "vision_time_adjuster.h",
    ],
    deps = [
        "//aos/containers:ring_buffer",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//y2017/control_loops/drivetrain:polydrivetrain_plants",
        "//y2017/vision:vision_fbs",
    ],
)

cc_test(
    name = "vision_time_adjuster_test",
    srcs = [
        "vision_time_adjuster_test.cc",
    ],
    data = ["//y2017:config.json"],
    deps = [
        ":vision_time_adjuster",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/time",
    ],
)

cc_library(
    name = "vision_distance_average",
    hdrs = [
        "vision_distance_average.h",
    ],
    deps = [
        "//aos/containers:ring_buffer",
        "//aos/time",
        "//y2017/vision:vision_fbs",
    ],
)

cc_test(
    name = "vision_distance_average_test",
    srcs = [
        "vision_distance_average_test.cc",
    ],
    deps = [
        ":vision_distance_average",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
        "//aos/time",
    ],
)
