load("//motors:macros.bzl", "hex_from_elf")
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
    name = "fet12.elf",
    srcs = [
        "current_equalization.h",
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
        "//motors/print:usb",
    ],
)

cc_binary(
    name = "fet12v2.elf",
    srcs = [
        "current_equalization.h",
        "fet12v2.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":motor_controls",
        "//motors:motor",
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:adc_dma",
        "//motors/peripheral:can",
        "//motors/print:itm",
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
        "@org_tuxfamily_eigen//:eigen",
    ],
)

py_library(
    name = "calib_sensors",
    srcs = [
        "calib_sensors.py",
    ],
    data = [
        "@python_repo//:scipy",
    ],
)

py_binary(
    name = "current_equalize",
    srcs = [
        "current_equalize.py",
    ],
    data = [
        ":calib_sensors",
        "@python_repo//:scipy",
    ],
)

genrule(
    name = "gen_current_equalization",
    srcs = [
        "calib_data_60a.csv",
        "calib_data_60b.csv",
        "calib_data_60c.csv",
        "calib_data_6030a.csv",
        "calib_data_6030b.csv",
        "calib_data_6030c.csv",
    ],
    outs = ["current_equalization.h"],
    cmd = " ".join([
        "$(location current_equalize)",
        "$(location calib_data_60a.csv)",
        "$(location calib_data_60b.csv)",
        "$(location calib_data_60c.csv)",
        "$(location calib_data_6030a.csv)",
        "$(location calib_data_6030b.csv)",
        "$(location calib_data_6030c.csv)",
        "> \"$@\"",
    ]),
    restricted_to = mcu_cpus,
    tools = ["current_equalize"],
)
