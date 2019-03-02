load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "adc",
    srcs = [
        "adc.cc",
    ],
    hdrs = [
        "adc.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        "//motors:util",
        "//motors/core",
    ],
)

cc_library(
    name = "configuration",
    hdrs = [
        "configuration.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "can",
    srcs = [
        "can.c",
    ],
    hdrs = [
        "can.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//motors:util",
        "//motors/core",
    ],
)

cc_library(
    name = "uart_buffer",
    hdrs = ["uart_buffer.h"],
    compatible_with = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/GSL",
    ],
)

cc_test(
    name = "uart_buffer_test",
    srcs = [
        "uart_buffer_test.cc",
    ],
    deps = [
        ":uart_buffer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "uart",
    srcs = ["uart.cc"],
    hdrs = ["uart.h"],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":uart_buffer",
        "//aos/containers:sized_array",
        "//motors:util",
        "//motors/core",
        "//third_party/GSL",
    ],
)

cc_library(
    name = "adc_dma",
    srcs = ["adc_dma.cc"],
    hdrs = ["adc_dma.h"],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        "//motors:util",
        "//motors/core",
    ],
)

cc_library(
    name = "spi",
    srcs = ["spi.cc"],
    hdrs = ["spi.h"],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":uart_buffer",
        "//motors:util",
        "//motors/core",
        "//third_party/GSL",
    ],
)
