load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")
load("@npm_bazel_typescript//:defs.bzl", "ts_library")

genrule(
    name = "genrule_drivetrain",
    outs = [
        "drivetrain_dog_motor_plant.h",
        "drivetrain_dog_motor_plant.cc",
        "kalman_drivetrain_motor_plant.h",
        "kalman_drivetrain_motor_plant.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:drivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2020/control_loops/python:drivetrain",
    ],
)

genrule(
    name = "genrule_polydrivetrain",
    outs = [
        "polydrivetrain_dog_motor_plant.h",
        "polydrivetrain_dog_motor_plant.cc",
        "polydrivetrain_cim_plant.h",
        "polydrivetrain_cim_plant.cc",
        "hybrid_velocity_drivetrain.h",
        "hybrid_velocity_drivetrain.cc",
    ],
    cmd = "$(location //y2020/control_loops/python:polydrivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2020/control_loops/python:polydrivetrain",
    ],
)

cc_library(
    name = "polydrivetrain_plants",
    srcs = [
        "drivetrain_dog_motor_plant.cc",
        "hybrid_velocity_drivetrain.cc",
        "kalman_drivetrain_motor_plant.cc",
        "polydrivetrain_dog_motor_plant.cc",
    ],
    hdrs = [
        "drivetrain_dog_motor_plant.h",
        "hybrid_velocity_drivetrain.h",
        "kalman_drivetrain_motor_plant.h",
        "polydrivetrain_dog_motor_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:hybrid_state_feedback_loop",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

flatbuffer_cc_library(
    name = "localizer_debug_fbs",
    srcs = ["localizer_debug.fbs"],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "localizer_debug_ts_fbs",
    srcs = ["localizer_debug.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "localizer",
    srcs = ["localizer.cc"],
    hdrs = ["localizer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":localizer_debug_fbs",
        "//aos/containers:ring_buffer",
        "//aos/containers:sized_array",
        "//aos/network:message_bridge_server_fbs",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:hybrid_ekf",
        "//frc971/control_loops/drivetrain:localizer",
        "//y2020:constants",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
        "//y2020/vision/sift:sift_fbs",
    ],
)

cc_library(
    name = "drivetrain_base",
    srcs = [
        "drivetrain_base.cc",
    ],
    hdrs = [
        "drivetrain_base.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":polydrivetrain_plants",
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//y2020:constants",
    ],
)

cc_binary(
    name = "trajectory_generator",
    srcs = [
        "trajectory_generator_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":drivetrain_base",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/drivetrain:trajectory_generator",
    ],
)

cc_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":drivetrain_base",
        ":localizer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
    ],
)

aos_config(
    name = "simulation_config",
    src = "drivetrain_simulation_config.json",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:simulation_channels",
        "//y2020:config",
    ],
)

cc_test(
    name = "localizer_test",
    srcs = ["localizer_test.cc"],
    data = [":simulation_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_base",
        ":localizer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/network:team_number",
        "//aos/network:testing_time_converter",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain:drivetrain_test_lib",
    ],
)

cc_test(
    name = "drivetrain_replay_test",
    srcs = ["drivetrain_replay_test.cc"],
    data = [
        "//y2020:config",
        "@drivetrain_replay",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_base",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain:trajectory_schema",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "drivetrain_replay",
    srcs = ["drivetrain_replay.cc"],
    data = ["//y2020:config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_base",
        ":localizer",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain:trajectory_generator",
        "//y2020:constants",
        "//y2020/control_loops/superstructure:superstructure_lib",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

ts_library(
    name = "localizer_plotter",
    srcs = ["localizer_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
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
