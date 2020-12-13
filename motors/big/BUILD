load("//motors:macros.bzl", "hex_from_elf")

cc_binary(
    name = "medium_salsa.elf",
    srcs = [
        "medium_salsa.cc",
    ],
    target_compatible_with = ["@platforms//os:none"],
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
    target_compatible_with = ["@platforms//os:none"],
)

cc_library(
    name = "motor_controls",
    srcs = [
        "motor_controls.cc",
    ],
    hdrs = [
        "motor_controls.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    deps = [
        "//motors:math",
        "//motors:motor",
        "//motors/peripheral:configuration",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
