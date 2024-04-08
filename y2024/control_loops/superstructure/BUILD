load("//tools/build_rules:js.bzl", "ts_project")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    deps = [
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/catapult:catapult_goal_fbs",
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
        "//frc971/control_loops/drivetrain:drivetrain_can_position_fbs",
    ],
)

flatbuffer_ts_library(
    name = "superstructure_position_ts_fbs",
    srcs = [
        "superstructure_position.fbs",
    ],
    deps = [
        "//frc971/control_loops:control_loops_ts_fbs",
        "//frc971/control_loops:profiled_subsystem_ts_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_can_position_ts_fbs",
    ],
)

static_flatbuffer(
    name = "superstructure_can_position_fbs",
    srcs = ["superstructure_can_position.fbs"],
    deps = ["//frc971/control_loops:can_talonfx_fbs"],
)

flatbuffer_ts_library(
    name = "superstructure_can_position_ts_fbs",
    srcs = ["superstructure_can_position.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//frc971/control_loops:can_talonfx_ts_fbs"],
)

cc_library(
    name = "superstructure_lib",
    srcs = [
        "superstructure.cc",
    ],
    hdrs = [
        "superstructure.h",
    ],
    data = [],
    deps = [
        ":collision_avoidance_lib",
        ":shooter",
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/shooter_interpolation:interpolation",
        "//frc971/zeroing:absolute_encoder",
        "//frc971/zeroing:pot_and_absolute_encoder",
        "//y2024:constants",
        "//y2024/constants:constants_fbs",
        "//y2024/constants:simulated_constants_sender",
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
    name = "collision_avoidance_lib",
    srcs = ["collision_avoidance.cc"],
    hdrs = ["collision_avoidance.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
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

cc_test(
    name = "superstructure_lib_test",
    srcs = [
        "superstructure_lib_test.cc",
    ],
    data = [
        "//y2024:aos_config",
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
    ],
)

cc_library(
    name = "shooter",
    srcs = [
        "shooter.cc",
    ],
    hdrs = [
        "shooter.h",
    ],
    deps = [
        ":aiming",
        ":collision_avoidance_lib",
        ":superstructure_can_position_fbs",
        ":superstructure_goal_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//frc971/control_loops/catapult",
        "//frc971/control_loops/catapult:catapult_goal_fbs",
        "//frc971/shooter_interpolation:interpolation",
        "//frc971/zeroing:pot_and_absolute_encoder",
        "//y2024:constants",
        "//y2024/constants:constants_fbs",
        "//y2024/control_loops/superstructure/altitude:altitude_plants",
        "//y2024/control_loops/superstructure/catapult:catapult_plants",
        "//y2024/control_loops/superstructure/turret:turret_plants",
    ],
)

cc_library(
    name = "aiming",
    srcs = [
        "aiming.cc",
    ],
    hdrs = [
        "aiming.h",
    ],
    deps = [
        ":superstructure_can_position_fbs",
        ":superstructure_goal_fbs",
        ":superstructure_status_fbs",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//frc971/control_loops/aiming",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc971/zeroing:averager",
        "//y2024:constants",
        "//y2024/constants:constants_fbs",
        "//y2024/control_loops/drivetrain:drivetrain_base",
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
    ],
)
