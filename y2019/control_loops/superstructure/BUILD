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
        ":collision_avoidance",
        ":superstructure_queue",
        ":vacuum",
        "//aos/controls:control_loop",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2019:constants",
        "//y2019:status_light",
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
        "//frc971/control_loops:capped_test_plant",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//y2019:status_light",
        "//y2019/control_loops/superstructure/intake:intake_plants",
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
        "//aos/events:shm-event-loop",
    ],
)

cc_library(
    name = "collision_avoidance",
    srcs = [
        "collision_avoidance.cc",
    ],
    hdrs = [
        "collision_avoidance.h",
    ],
    deps = [
        ":superstructure_queue",
        "//aos/controls:control_loop_queues",
        "//frc971:constants",
    ],
)

cc_library(
    name = "vacuum",
    srcs = [
        "vacuum.cc",
    ],
    hdrs = [
        "vacuum.h",
    ],
    deps = [
        ":superstructure_queue",
        "//aos/controls:control_loop",
    ],
)

cc_test(
    name = "collision_avoidance_tests",
    srcs = [
        "collision_avoidance_tests.cc",
    ],
    deps = [
        ":collision_avoidance",
        ":superstructure_queue",
        "//aos:math",
        "//aos/testing:googletest",
    ],
)
