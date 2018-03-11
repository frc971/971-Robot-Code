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
        "//aos/common:mutex",
        "//aos/common/logging",
        "//aos/linux_code:init",
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
        "//aos/common/logging",
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
        "//aos/common:stl_mutex",
        "//aos/common:time",
        "//aos/common/logging",
        "//aos/linux_code:init",
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
        "//aos/common/logging",
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
        "//aos/common:time",
        "//aos/common/logging",
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
        "//aos/common:time",
        "//aos/common/logging",
        "//aos/common/logging:queue_logging",
        "//aos/common/messages:robot_state",
        "//aos/common/util:phased_loop",
        "//aos/linux_code:init",
        "//frc971/queues:gyro",
        "//frc971/zeroing:averager",
    ],
)

cc_library(
    name = "lpd8806",
    srcs = [
        "LPD8806.cc",
    ],
    hdrs = [
        "LPD8806.h",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//aos/common:mutex",
        "//aos/common:time",
        "//frc971/queues:gyro",
        "//third_party:wpilib",
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
        "//aos/common:scoped_fd",
        "//aos/common:time",
        "//aos/common/messages:robot_state",
        "//aos/common/util:log_interval",
        "//aos/linux_code:init",
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
        "//aos/common/logging:queue_logging",
        "//aos/common/messages:robot_state",
        "//aos/common/network:team_number",
        "//aos/linux_code:init",
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
        "//aos/common/logging:queue_logging",
        "//aos/common/messages:robot_state",
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
        "//aos/common/logging:queue_logging",
        "//aos/common/util:phased_loop",
        "//aos/linux_code:init",
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
        "//aos/common:time",
        "//aos/common/logging",
        "//aos/common/logging:queue_logging",
        "//aos/common/messages:robot_state",
        "//aos/linux_code:init",
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
        "//aos/common/logging",
    ],
)
