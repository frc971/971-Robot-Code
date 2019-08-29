load("//motors:macros.bzl", "hex_from_elf")
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
    name = "medium_salsa.elf",
    srcs = [
        "medium_salsa.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":motor_controls",
        "//motors:motor",
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/usb:legacy",
    ],
)

hex_from_elf(
    name = "medium_salsa",
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
        "@org_tuxfamily_eigen//:eigen",
    ],
)
