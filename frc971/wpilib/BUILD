package(default_visibility = ["//visibility:public"])

load("//aos/build:queues.bzl", "queue_library")

queue_library(
    name = "logging_queue",
    srcs = [
        "logging.q",
    ],
)

cc_library(
    name = "encoder_and_potentiometer",
    srcs = [
        "encoder_and_potentiometer.cc",
    ],
    hdrs = [
        "encoder_and_potentiometer.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":dma",
        ":dma_edge_counting",
        "//aos:init",
        "//aos/logging",
        "//aos/mutex",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "dma_edge_counting",
    srcs = [
        "dma_edge_counting.cc",
    ],
    hdrs = [
        "dma_edge_counting.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":dma",
        "//aos/logging",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "interrupt_edge_counting",
    srcs = [
        "interrupt_edge_counting.cc",
    ],
    hdrs = [
        "interrupt_edge_counting.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos:init",
        "//aos/logging",
        "//aos/stl_mutex",
        "//aos/time",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "buffered_pcm",
    srcs = [
        "buffered_pcm.cc",
        "buffered_solenoid.cc",
    ],
    hdrs = [
        "buffered_pcm.h",
        "buffered_solenoid.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos/logging",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "gyro_interface",
    srcs = [
        "gyro_interface.cc",
    ],
    hdrs = [
        "gyro_interface.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos/logging",
        "//aos/time",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "gyro_sender",
    srcs = [
        "gyro_sender.cc",
    ],
    hdrs = [
        "gyro_sender.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":gyro_interface",
        "//aos:init",
        "//aos/events:event-loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/time",
        "//aos/util:phased_loop",
        "//frc971/queues:gyro",
        "//frc971/zeroing:averager",
    ],
)

queue_library(
    name = "loop_output_handler_test_queue",
    srcs = [
        "loop_output_handler_test.q",
    ],
)

cc_library(
    name = "loop_output_handler",
    srcs = [
        "loop_output_handler.cc",
    ],
    hdrs = [
        "loop_output_handler.h",
    ],
    deps = [
        "//aos:init",
        "//aos/events:event-loop",
        "//aos/robot_state",
        "//aos/scoped:scoped_fd",
        "//aos/time",
        "//aos/util:log_interval",
    ],
)

cc_test(
    name = "loop_output_handler_test",
    srcs = [
        "loop_output_handler_test.cc",
    ],
    deps = [
        ":loop_output_handler",
        ":loop_output_handler_test_queue",
        "//aos/events:simulated_event_loop",
        "//aos/logging:queue_logging",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "joystick_sender",
    srcs = [
        "joystick_sender.cc",
    ],
    hdrs = [
        "joystick_sender.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos:init",
        "//aos/events:shm-event-loop",
        "//aos/logging:queue_logging",
        "//aos/network:team_number",
        "//aos/robot_state",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    hdrs = [
        "wpilib_interface.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//third_party:wpilib",
    ],
)

queue_library(
    name = "pdp_values",
    srcs = [
        "pdp_values.q",
    ],
)

cc_library(
    name = "pdp_fetcher",
    srcs = [
        "pdp_fetcher.cc",
    ],
    hdrs = [
        "pdp_fetcher.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":pdp_values",
        "//aos:init",
        "//aos/events:event-loop",
        "//aos/logging:queue_logging",
        "//aos/util:phased_loop",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "wpilib_robot_base",
    hdrs = [
        "wpilib_robot_base.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//third_party:wpilib",
    ],
)

queue_library(
    name = "imu_queue",
    srcs = [
        "imu.q",
    ],
)

cc_library(
    name = "ADIS16448",
    srcs = [
        "ADIS16448.cc",
    ],
    hdrs = [
        "ADIS16448.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":imu_queue",
        ":spi_rx_clearer",
        "//aos:init",
        "//aos/events:event-loop",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//aos/robot_state",
        "//aos/time",
        "//frc971/zeroing:averager",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "dma",
    srcs = [
        "dma.cc",
    ],
    hdrs = [
        "dma.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "spi_rx_clearer",
    srcs = [
        "spi_rx_clearer.cc",
    ],
    hdrs = [
        "spi_rx_clearer.h",
    ],
    deps = [
        "//aos/logging",
    ],
)

cc_library(
    name = "sensor_reader",
    srcs = [
        "sensor_reader.cc",
    ],
    hdrs = [
        "sensor_reader.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":dma",
        ":dma_edge_counting",
        ":encoder_and_potentiometer",
        ":wpilib_interface",
        "//aos:init",
        "//aos/events:event-loop",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:phased_loop",
        "//frc971/control_loops:queues",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "drivetrain_writer",
    srcs = [
        "drivetrain_writer.cc",
    ],
    hdrs = [
        "drivetrain_writer.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":loop_output_handler",
        "//aos:math",
        "//aos/logging",
        "//aos/logging:queue_logging",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//third_party:wpilib",
    ],
)
