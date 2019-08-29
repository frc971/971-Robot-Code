load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools/build_rules:select.bzl", "compiler_select", "cpu_select")

genrule(
    name = "genrule_drivetrain",
    outs = [
        "drivetrain_dog_motor_plant.h",
        "drivetrain_dog_motor_plant.cc",
        "kalman_drivetrain_motor_plant.h",
        "kalman_drivetrain_motor_plant.cc",
    ],
    cmd = "$(location //y2019/control_loops/python:drivetrain) $(OUTS)",
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
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "camera_fbs",
    srcs = [
        "camera.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "camera",
    srcs = ["camera.h"],
    visibility = ["//y2019:__pkg__"],
    deps = [
        "//aos/containers:sized_array",
        "//frc971/control_loops:pose",
    ],
)

cc_test(
    name = "camera_test",
    srcs = ["camera_test.cc"],
    deps = [
        ":camera",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "localizer",
    hdrs = ["localizer.h"],
    deps = [
        ":camera",
        "//frc971/control_loops:pose",
        "//frc971/control_loops/drivetrain:hybrid_ekf",
    ],
)

cc_library(
    name = "target_selector",
    srcs = ["target_selector.cc"],
    hdrs = ["target_selector.h"],
    deps = [
        ":camera",
        ":target_selector_fbs",
        "//frc971/control_loops:pose",
        "//frc971/control_loops/drivetrain:localizer",
        "//y2019:constants",
        "//y2019/control_loops/superstructure:superstructure_goal_fbs",
    ],
)

cc_test(
    name = "target_selector_test",
    srcs = ["target_selector_test.cc"],
    data = ["//y2019:config.json"],
    deps = [
        ":target_selector",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
    ],
)

cc_library(
    name = "event_loop_localizer",
    srcs = ["event_loop_localizer.cc"],
    hdrs = ["event_loop_localizer.h"],
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
    shard_count = 5,
    deps = [
        ":localizer",
        ":drivetrain_base",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "//aos/testing:test_shm",
        "//frc971/control_loops/drivetrain:trajectory",
        "//y2019:constants",
        "//frc971/control_loops/drivetrain:splinedrivetrain",
        "@com_github_gflags_gflags//:gflags",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

cc_test(
    name = "localized_drivetrain_test",
    srcs = ["localized_drivetrain_test.cc"],
    data = ["//y2019:config.json"],
    deps = [
        ":camera_fbs",
        ":drivetrain_base",
        ":event_loop_localizer",
        ":localizer",
        "//aos/controls:control_loop_test",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_lib",
        "//frc971/control_loops/drivetrain:drivetrain_test_lib",
    ],
)
