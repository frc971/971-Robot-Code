load("//motors:macros.bzl", "hex_from_elf")
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
    name = "medium_salsa.elf",
    srcs = [
        "medium_salsa.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":motor",
        ":motor_controls",
        ":util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/usb:legacy",
    ],
)

cc_library(
    name = "motor",
    srcs = [
        "motor.cc",
    ],
    hdrs = [
        "motor.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":algorithms",
        ":util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/peripheral:configuration",
        "//motors/usb:cdc",
    ],
)

hex_from_elf(
    name = "medium_salsa",
    restricted_to = mcu_cpus,
)

cc_library(
    name = "util",
    hdrs = [
        "util.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//motors/core",
    ],
)

genrule(
    name = "doc",
    srcs = [
        "NOTES.md",
    ],
    outs = [
        "NOTES.html",
    ],
    cmd = " ".join([
        "pandoc",
        "-f",
        "markdown_github-hard_line_breaks",
        "-t",
        "html5",
        "-o",
        "$@",
        "$<",
    ]),
)

cc_library(
    name = "algorithms",
    srcs = [
        "algorithms.cc",
    ],
    hdrs = [
        "algorithms.h",
    ],
    compatible_with = mcu_cpus,
)

cc_test(
    name = "algorithms_test",
    srcs = [
        "algorithms_test.cc",
    ],
    deps = [
        ":algorithms",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "math",
    srcs = [
        "math.cc",
    ],
    hdrs = [
        "math.h",
    ],
    compatible_with = mcu_cpus,
    visibility = ["//visibility:public"],
)

cc_test(
    name = "math_test",
    srcs = [
        "math_test.cc",
    ],
    deps = [
        ":math",
        "//aos/testing:googletest",
        "//third_party/googletest:googlemock",
    ],
)

cc_library(
    name = "motor_controls",
    srcs = [
        "motor_controls.cc",
    ],
    hdrs = [
        "motor_controls.h",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":math",
        ":motor",
        "//motors/peripheral:configuration",
        "//third_party/eigen",
    ],
)

cc_binary(
    name = "button_board.elf",
    srcs = [
        "button_board.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/usb",
        "//motors/usb:cdc",
        "//motors/usb:hid",
    ],
)

hex_from_elf(
    name = "button_board",
    restricted_to = mcu_cpus,
)

cc_binary(
    name = "simple_receiver.elf",
    srcs = [
        "simple_receiver.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/usb",
        "//motors/usb:cdc",
        "//motors/seems_reasonable:drivetrain_lib",
    ],
)

hex_from_elf(
    name = "simple_receiver",
    restricted_to = mcu_cpus,
)
