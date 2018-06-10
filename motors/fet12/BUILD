load("//motors:macros.bzl", "hex_from_elf")
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
    name = "fet12.elf",
    srcs = [
        "fet12.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":motor_controls",
        "//motors:motor",
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/usb",
        "//motors/usb:cdc",
    ],
)

hex_from_elf(
    name = "fet12",
    restricted_to = mcu_cpus,
)

cc_binary(
    name = "power_wheels.elf",
    srcs = [
        "power_wheels.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/usb",
        "//motors/usb:cdc",
    ],
)

hex_from_elf(
    name = "power_wheels",
    restricted_to = mcu_cpus,
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
        "//motors:math",
        "//motors:motor",
        "//motors/peripheral:configuration",
        "//third_party/eigen",
    ],
)
