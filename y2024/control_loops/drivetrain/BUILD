load("//aos:config.bzl", "aos_config")
load("//frc971/control_loops/drivetrain:drivetrain_config.bzl", "drivetrain_config")

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
        "//frc971/control_loops/drivetrain:drivetrain_config",
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
        "//frc971/control_loops/drivetrain:trajectory_generator",
    ],
)

drivetrain_config(
    name = "drivetrain_config",
    out = "drivetrain_config.json",
)
