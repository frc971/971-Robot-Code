load("//tools/build_rules:js.bzl", "ts_project")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    gen_reflections = 1,
    deps = [
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
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
    deps = [
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//y2023/vision:game_pieces_fbs",
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
        "//y2023/vision:game_pieces_ts_fbs",
    ],
)

flatbuffer_cc_library(
    name = "superstructure_position_fbs",
    srcs = [
        "superstructure_position.fbs",
    ],
    gen_reflections = 1,
    deps = [
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/vision:calibration_fbs",
        "//y2023/control_loops/drivetrain:drivetrain_can_position_fbs",
    ],
)

cc_library(
    name = "end_effector",
    srcs = [
        "end_effector.cc",
    ],
    hdrs = [
        "end_effector.h",
    ],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//aos/events:event_loop",
        "//aos/time",
        "//frc971/control_loops:control_loop",
        "//y2023:constants",
        "//y2023/vision:game_pieces_fbs",
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
    data = [
        "//y2023/control_loops/superstructure/arm:arm_trajectories_generated.bfbs",
    ],
    deps = [
        ":end_effector",
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/shooter_interpolation:interpolation",
        "//y2023:constants",
        "//y2023/constants:constants_fbs",
        "//y2023/constants:simulated_constants_sender",
        "//y2023/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//y2023/control_loops/superstructure/arm",
        "//y2023/control_loops/superstructure/arm:arm_trajectories_fbs",
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
        "//y2023:aos_config",
    ],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_lib",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:json_to_flatbuffer",
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
        "//y2023/control_loops/superstructure/roll:roll_plants",
        "//y2023/vision:game_pieces_fbs",
    ],
)

cc_library(
    name = "led_indicator_lib",
    srcs = ["led_indicator.cc"],
    hdrs = ["led_indicator.h"],
    data = [
        "@ctre_phoenix_api_cpp_athena//:shared_libraries",
        "@ctre_phoenix_cci_athena//:shared_libraries",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_output_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc971/queues:gyro_fbs",
        "//third_party:phoenix",
        "//third_party:wpilib",
        "//y2023/vision:game_pieces_fbs",
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

ts_project(
    name = "superstructure_plotter",
    srcs = ["superstructure_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)
