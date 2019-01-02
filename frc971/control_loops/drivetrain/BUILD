package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
    name = "replay_drivetrain",
    srcs = [
        "replay_drivetrain.cc",
    ],
    deps = [
        ":drivetrain_queue",
        "//aos:init",
        "//aos/controls:replay_control_loop",
        "//frc971/queues:gyro",
    ],
)

queue_library(
    name = "drivetrain_queue",
    srcs = [
        "drivetrain.q",
    ],
    deps = [
        "//aos/controls:control_loop_queues",
        "//frc971/control_loops:queues",
    ],
)

cc_library(
    name = "drivetrain_config",
    hdrs = [
        "drivetrain_config.h",
    ],
    deps = [
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "gear",
    hdrs = [
        "gear.h",
    ],
    compatible_with = mcu_cpus,
)

cc_library(
    name = "ssdrivetrain",
    srcs = [
        "ssdrivetrain.cc",
    ],
    hdrs = [
        "ssdrivetrain.h",
    ],
    deps = [
        ":drivetrain_config",
        ":drivetrain_queue",
        ":gear",
        "//aos:math",
        "//aos/controls:control_loop",
        "//aos/controls:polytope",
        "//aos/logging:matrix_logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/util:log_interval",
        "//aos/util:trapezoid_profile",
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "polydrivetrain",
    srcs = [
        "polydrivetrain.cc",
    ],
    hdrs = [
        "polydrivetrain.h",
    ],
    deps = [
        ":drivetrain_config",
        ":drivetrain_queue",
        ":gear",
        "//aos:math",
        "//aos/controls:polytope",
        "//aos/logging:matrix_logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/util:log_interval",
        "//frc971/control_loops:coerce_goal",
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "drivetrain_config_uc",
    hdrs = [
        "drivetrain_config.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//frc971:shifter_hall_effect",
        "//frc971/control_loops:state_feedback_loop_uc",
    ],
)

cc_library(
    name = "polydrivetrain_uc",
    srcs = [
        "drivetrain_uc.q.cc",
        "polydrivetrain.cc",
    ],
    hdrs = [
        "drivetrain_uc.q.h",
        "polydrivetrain.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":drivetrain_config_uc",
        ":gear",
        "//aos:math",
        "//aos/controls:polytope_uc",
        "//frc971/control_loops:coerce_goal_uc",
        "//frc971/control_loops:state_feedback_loop_uc",
    ],
)

genrule(
    name = "genrule_down_estimator",
    outs = [
        "down_estimator.h",
        "down_estimator.cc",
    ],
    cmd = "$(location //frc971/control_loops/python:down_estimator) $(OUTS)",
    tools = [
        "//frc971/control_loops/python:down_estimator",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "down_estimator",
    srcs = [
        "down_estimator.cc",
    ],
    hdrs = [
        "down_estimator.h",
    ],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "drivetrain_lib",
    srcs = [
        "drivetrain.cc",
    ],
    hdrs = [
        "drivetrain.h",
    ],
    deps = [
        ":down_estimator",
        ":drivetrain_queue",
        ":gear",
        ":polydrivetrain",
        ":ssdrivetrain",
        "//aos/controls:control_loop",
        "//aos/logging:matrix_logging",
        "//aos/logging:queue_logging",
        "//aos/util:log_interval",
        "//frc971/queues:gyro",
        "//frc971/wpilib:imu_queue",
    ],
)

cc_test(
    name = "drivetrain_lib_test",
    srcs = [
        "drivetrain_lib_test.cc",
    ],
    deps = [
        ":drivetrain_config",
        ":drivetrain_lib",
        ":drivetrain_queue",
        "//aos:queues",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/queues:gyro",
        "//y2016:constants",
        "//y2016/control_loops/drivetrain:polydrivetrain_plants",
    ],
)

genrule(
    name = "genrule_haptic_wheel",
    outs = [
        "haptic_wheel.h",
        "haptic_wheel.cc",
        "integral_haptic_wheel.h",
        "integral_haptic_wheel.cc",
        "haptic_trigger.h",
        "haptic_trigger.cc",
        "integral_haptic_trigger.h",
        "integral_haptic_trigger.cc",
    ],
    cmd = "$(location //frc971/control_loops/python:haptic_wheel) $(OUTS)",
    compatible_with = mcu_cpus,
    tools = [
        "//frc971/control_loops/python:haptic_wheel",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "haptic_input_uc",
    srcs = [
        "haptic_trigger.cc",
        "haptic_wheel.cc",
        "integral_haptic_trigger.cc",
        "integral_haptic_wheel.cc",
    ],
    hdrs = [
        "haptic_trigger.h",
        "haptic_wheel.h",
        "integral_haptic_trigger.h",
        "integral_haptic_wheel.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//frc971/control_loops:state_feedback_loop_uc",
    ],
)

cc_library(
    name = "haptic_wheel",
    srcs = [
        "haptic_trigger.cc",
        "haptic_wheel.cc",
        "integral_haptic_trigger.cc",
        "integral_haptic_wheel.cc",
    ],
    hdrs = [
        "haptic_trigger.h",
        "haptic_wheel.h",
        "integral_haptic_trigger.h",
        "integral_haptic_wheel.h",
    ],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "spline",
    srcs = ["spline.cc"],
    hdrs = ["spline.h"],
    deps = [
        "//third_party/eigen",
    ],
)

cc_test(
    name = "spline_test",
    srcs = [
        "spline_test.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":spline",
        "//aos/testing:googletest",
        "//third_party/matplotlib-cpp",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "distance_spline",
    srcs = ["distance_spline.cc"],
    hdrs = ["distance_spline.h"],
    deps = [
        ":spline",
        "//frc971/control_loops:fixed_quadrature",
        "//third_party/eigen",
    ],
)

cc_test(
    name = "distance_spline_test",
    srcs = [
        "distance_spline_test.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        ":distance_spline",
        "//aos/testing:googletest",
        "//third_party/matplotlib-cpp",
        "@com_github_gflags_gflags//:gflags",
    ],
)
