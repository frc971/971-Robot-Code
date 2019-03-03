load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "print",
    hdrs = [
        "print.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//aos/containers:sized_array",
        "//motors/core",
        "//third_party/GSL",
    ],
)

cc_library(
    name = "uart",
    srcs = [
        "uart.cc",
    ],
    hdrs = [
        "uart.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core",
        "//motors/peripheral:uart",
    ],
)

cc_library(
    name = "itm",
    srcs = [
        "itm.cc",
    ],
    hdrs = [
        "itm.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core:itm",
    ],
)

cc_library(
    name = "semihosting",
    srcs = [
        "semihosting.cc",
    ],
    hdrs = [
        "semihosting.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core:semihosting",
    ],
)

cc_library(
    name = "usb",
    srcs = [
        "usb.cc",
    ],
    hdrs = [
        "usb.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":print",
        "//motors/core",
        "//motors/usb",
        "//motors/usb:cdc",
    ],
)
