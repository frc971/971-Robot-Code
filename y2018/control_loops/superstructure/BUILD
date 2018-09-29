package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "superstructure_queue",
    srcs = [
        "superstructure.q",
    ],
    deps = [
        "//aos/controls:control_loop_queues",
        "//frc971/control_loops:queues",
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
        ":superstructure_queue",
        "//aos/controls:control_loop",
        "//frc971/control_loops:queues",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2018:constants",
        "//y2018:status_light",
        "//y2018/control_loops/superstructure/arm",
        "//y2018/control_loops/superstructure/intake",
        "//y2018/vision:vision_queue",
    ],
)

cc_test(
    name = "superstructure_lib_test",
    srcs = [
        "superstructure_lib_test.cc",
    ],
    deps = [
        ":superstructure_lib",
        ":superstructure_queue",
        "//aos:math",
        "//aos:queues",
        "//aos/time:time",
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//y2018/control_loops/superstructure/intake:intake_plants",
    ],
)

cc_binary(
    name = "superstructure",
    srcs = [
        "superstructure_main.cc",
    ],
    deps = [
        ":superstructure_lib",
        ":superstructure_queue",
        "//aos/linux_code:init",
    ],
)

cc_library(
    name = "debouncer",
    hdrs = [
        "debouncer.h",
    ],
    srcs = [
        "debouncer.cc",
    ],
)

cc_test(
    name = "debouncer_test",
    srcs = [
        "debouncer_test.cc",
    ],
    deps = [
        ":debouncer",
        "//aos/testing:googletest",
    ],
)
