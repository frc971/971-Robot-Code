package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "superstructure_queue",
    srcs = [
        "superstructure.q",
    ],
    deps = [
        "//aos/controls:control_loop_queues",
        "//frc971/control_loops:profiled_subsystem_queue",
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
        ":vision_distance_average",
        "//aos/controls:control_loop",
        "//y2017:constants",
        "//y2017/control_loops/superstructure/column",
        "//y2017/control_loops/superstructure/hood",
        "//y2017/control_loops/superstructure/intake",
        "//y2017/control_loops/superstructure/shooter",
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
        "//aos/controls:control_loop_test",
        "//aos/testing:googletest",
        "//aos/time",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//y2017/control_loops/superstructure/column:column_plants",
        "//y2017/control_loops/superstructure/hood:hood_plants",
        "//y2017/control_loops/superstructure/intake:intake_plants",
        "//y2017/control_loops/superstructure/shooter:shooter_plants",
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
        "//aos:init",
    ],
)

cc_library(
    name = "vision_time_adjuster",
    srcs = [
        "vision_time_adjuster.cc",
    ],
    hdrs = [
        "vision_time_adjuster.h",
    ],
    deps = [
        ":superstructure_queue",
        "//aos/containers:ring_buffer",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2017/control_loops/drivetrain:polydrivetrain_plants",
        "//y2017/vision:vision_queue",
    ],
)

cc_test(
    name = "vision_time_adjuster_test",
    srcs = [
        "vision_time_adjuster_test.cc",
    ],
    deps = [
        ":vision_time_adjuster",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/time",
    ],
)

cc_library(
    name = "vision_distance_average",
    hdrs = [
        "vision_distance_average.h",
    ],
    deps = [
        "//aos/containers:ring_buffer",
        "//aos/time",
        "//y2017/vision:vision_queue",
    ],
)

cc_test(
    name = "vision_distance_average_test",
    srcs = [
        "vision_distance_average_test.cc",
    ],
    deps = [
        ":vision_distance_average",
        "//aos/testing:googletest",
        "//aos/time",
    ],
)
