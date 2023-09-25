load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")
load("//tools/build_rules:select.bzl", "cpu_select")

genrule(
    name = "genrule_drivetrain",
    outs = [
        "drivetrain_dog_motor_plant.h",
        "drivetrain_dog_motor_plant.cc",
        "kalman_drivetrain_motor_plant.h",
        "kalman_drivetrain_motor_plant.cc",
    ],
    cmd = "$(location //y2019/control_loops/python:drivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2019/control_loops/python:drivetrain",
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
    cmd = "$(location //y2019/control_loops/python:polydrivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//y2019/control_loops/python:polydrivetrain",
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
        ":event_loop_localizer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
    ],
)

flatbuffer_cc_library(
    name = "target_selector_fbs",
    srcs = ["target_selector.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "camera_fbs",
    srcs = [
        "camera.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "localizer",
    hdrs = ["localizer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc971/control_loops:pose",
        "//frc971/control_loops/drivetrain:camera",
        "//frc971/control_loops/drivetrain:hybrid_ekf",
    ],
)

cc_library(
    name = "target_selector",
    srcs = ["target_selector.cc"],
    hdrs = ["target_selector.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":target_selector_fbs",
        "//frc971/control_loops:pose",
        "//frc971/control_loops/drivetrain:camera",
        "//frc971/control_loops/drivetrain:localizer",
        "//y2019:constants",
        "//y2019/control_loops/superstructure:superstructure_goal_fbs",
    ],
)

cc_test(
    name = "target_selector_test",
    srcs = ["target_selector_test.cc"],
    data = ["//y2019:aos_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":target_selector",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "event_loop_localizer",
    srcs = ["event_loop_localizer.cc"],
    hdrs = ["event_loop_localizer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera_fbs",
        ":localizer",
        ":target_selector",
        "//frc971/control_loops/drivetrain:localizer",
        "//y2019:constants",
    ],
)

cc_test(
    name = "localizer_test",
    srcs = ["localizer_test.cc"],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    shard_count = 8,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_base",
        ":localizer",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "//aos/testing:test_shm",
        "//frc971/control_loops/drivetrain:splinedrivetrain",
        "//frc971/control_loops/drivetrain:trajectory",
        "//y2019:constants",
        "@com_github_gflags_gflags//:gflags",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

aos_config(
    name = "simulation_config",
    src = "drivetrain_simulation_config.json",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops/drivetrain:simulation_channels",
        "//y2019:aos_config",
    ],
)

cc_test(
    name = "localized_drivetrain_test",
    srcs = ["localized_drivetrain_test.cc"],
    data = [":simulation_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera_fbs",
        ":drivetrain_base",
        ":event_loop_localizer",
        ":localizer",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "//frc971/control_loops:control_loop_test",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain:drivetrain_test_lib",
    ],
)

cc_binary(
    name = "drivetrain_replay",
    srcs = ["drivetrain_replay.cc"],
    data = ["//y2019:aos_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_base",
        ":event_loop_localizer",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)
