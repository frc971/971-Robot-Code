cc_library(
    name = "adc",
    srcs = [
        "adc.cc",
    ],
    hdrs = [
        "adc.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
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
    target_compatible_with = ["@platforms//os:none"],
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
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        "//motors:util",
        "//motors/core",
    ],
)

cc_library(
    name = "uart_buffer",
    hdrs = ["uart_buffer.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "uart_buffer_test",
    srcs = [
        "uart_buffer_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":uart_buffer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "uart",
    srcs = ["uart.cc"],
    hdrs = ["uart.h"],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        ":uart_buffer",
        "//aos/containers:sized_array",
        "//motors:util",
        "//motors/core",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "adc_dma",
    srcs = ["adc_dma.cc"],
    hdrs = ["adc_dma.h"],
    target_compatible_with = ["@platforms//os:none"],
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
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        ":uart_buffer",
        "//motors:util",
        "//motors/core",
        "@com_google_absl//absl/types:span",
    ],
)
