load("//motors:macros.bzl", "hex_from_elf")
load("//tools:environments.bzl", "mcu_cpus")

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
