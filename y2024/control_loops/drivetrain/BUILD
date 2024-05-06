load("//aos:config.bzl", "aos_config")
load("//frc971/control_loops/drivetrain:drivetrain_config.bzl", "drivetrain_config")
load("//tools/build_rules:template.bzl", "jinja2_template")

genrule(
    name = "genrule_drivetrain",
    outs = [
        "drivetrain_dog_motor_plant.h",
        "drivetrain_dog_motor_plant.cc",
        "drivetrain_dog_motor_plant.json",
        "kalman_drivetrain_motor_plant.h",
        "kalman_drivetrain_motor_plant.cc",
        "kalman_drivetrain_motor_plant.json",
    ],
    cmd = "$(location //y2024/control_loops/python:drivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024/control_loops/python:drivetrain",
    ],
)

genrule(
    name = "genrule_polydrivetrain",
    outs = [
        "polydrivetrain_dog_motor_plant.h",
        "polydrivetrain_dog_motor_plant.cc",
        "polydrivetrain_dog_motor_plant.json",
        "polydrivetrain_cim_plant.h",
        "polydrivetrain_cim_plant.cc",
        "polydrivetrain_cim_plant.json",
        "hybrid_velocity_drivetrain.h",
        "hybrid_velocity_drivetrain.cc",
        "hybrid_velocity_drivetrain.json",
    ],
    cmd = "$(location //y2024/control_loops/python:polydrivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2024/control_loops/python:polydrivetrain",
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
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/drivetrain:drivetrain_config",
        "//y2024/constants:constants_fbs",
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
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain/localization:puppet_localizer",
        "//y2024/constants:constants_fbs",
    ],
)

aos_config(
    name = "simulation_config",
    src = "drivetrain_simulation_config.json",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:simulation_channels",
        "//y2024:aos_config",
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
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/drivetrain:trajectory_generator",
        "//y2024/constants:constants_fbs",
    ],
)

drivetrain_config(
    name = "drivetrain_loop_config",
    out = "drivetrain_loop_config.json",
)

jinja2_template(
    name = "drivetrain_config.json",
    src = "drivetrain_config.jinja2.json",
    includes = [
        ":drivetrain_loop_config.json",
    ],
    parameters = {},
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "drivetrain_replay",
    srcs = ["drivetrain_replay.cc"],
    data = [
        "//y2024:aos_config",
        "//y2024/constants:constants.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//aos/util:simulation_logger",
        "//frc971/constants:constants_sender_lib",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain:trajectory_generator",
        "//frc971/control_loops/drivetrain/localization:puppet_localizer",
        "//frc971/imu_fdcan:dual_imu_blender_lib",
        "//y2024/constants:simulated_constants_sender",
        "//y2024/control_loops/drivetrain:drivetrain_base",
    ],
)
