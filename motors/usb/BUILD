load("//tools:environments.bzl", "mcu_cpus")

cc_library(
    name = "legacy",
    srcs = [
        "usb_desc.c",
        "usb_dev.c",
        "usb_mem.c",
        "usb_mem.h",
        "usb_names.h",
        "usb_serial.c",
    ],
    hdrs = [
        "usb_desc.h",
        "usb_dev.h",
        "usb_serial.h",
    ],
    defines = [
        "USB_SERIAL=1",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//motors/core",
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
        ":constants",
        "//aos:macros",
        "//motors:util",
        "//motors/core",
    ],
)

cc_library(
    name = "cdc",
    srcs = [
        "cdc.cc",
    ],
    hdrs = [
        "cdc.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":queue",
        ":usb",
        "//motors:util",
        "//motors/core",
    ],
)

cc_library(
    name = "interrupt_out",
    srcs = [
        "interrupt_out.cc",
    ],
    hdrs = [
        "interrupt_out.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":usb",
        "//motors:util",
        "//motors/core",
    ],
)

cc_library(
    name = "queue",
    srcs = [
        "queue.cc",
    ],
    hdrs = [
        "queue.h",
    ],
    compatible_with = mcu_cpus,
)

cc_test(
    name = "queue_test",
    srcs = [
        "queue_test.cc",
    ],
    deps = [
        ":queue",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "constants",
    hdrs = [
        "constants.h",
    ],
    compatible_with = mcu_cpus,
)

cc_test(
    name = "constants_test",
    srcs = [
        "constants_test.cc",
    ],
    deps = [
        ":constants",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "hid",
    srcs = [
        "hid.cc",
    ],
    hdrs = [
        "hid.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":usb",
        "//motors:util",
    ],
)
