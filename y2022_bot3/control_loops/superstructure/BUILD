load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    deps = [
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
    ],
)

static_flatbuffer(
    name = "superstructure_output_fbs",
    srcs = [
        "superstructure_output.fbs",
    ],
)

static_flatbuffer(
    name = "superstructure_status_fbs",
    srcs = [
        "superstructure_status.fbs",
    ],
    deps = [
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
    ],
)

flatbuffer_ts_library(
    name = "superstructure_status_ts_fbs",
    srcs = [
        "superstructure_status.fbs",
    ],
    deps = [
        "//frc971/control_loops:control_loops_ts_fbs",
        "//frc971/control_loops:profiled_subsystem_ts_fbs",
    ],
)

static_flatbuffer(
    name = "superstructure_position_fbs",
    srcs = [
        "superstructure_position.fbs",
    ],
    deps = [
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
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
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/zeroing:pot_and_absolute_encoder",
        "//y2022_bot3:constants",
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
        "//y2022_bot3:aos_config",
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
        "//frc971/control_loops:subsystem_simulator",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
    ],
)

cc_binary(
    name = "superstructure_replay",
    srcs = ["superstructure_replay.cc"],
    deps = [
        ":superstructure_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/network:team_number",
    ],
)
