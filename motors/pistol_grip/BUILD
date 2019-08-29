load("//motors:macros.bzl", "hex_from_elf")
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
    name = "drivers_station.elf",
    srcs = [
        "drivers_station.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:can",
        "//motors/print:usb",
        "//motors/usb",
        "//motors/usb:cdc",
        "//motors/usb:hid",
        "//motors/usb:interrupt_out",
    ],
)

hex_from_elf(
    name = "drivers_station",
    restricted_to = mcu_cpus,
)

cc_binary(
    name = "controller.elf",
    srcs = [
        "controller.cc",
        "controller_adc.cc",
        "controller_adc.h",
        "vtable_trigger0.cc",
        "vtable_trigger1.cc",
        "vtable_wheel0.cc",
        "vtable_wheel1.cc",
    ],
    restricted_to = mcu_cpus,
    deps = [
        ":motor_controls",
        "//frc971/control_loops/drivetrain:haptic_input_uc",
        "//motors:motor",
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/print:usb",
    ],
)

hex_from_elf(
    name = "controller",
    restricted_to = mcu_cpus,
)

cc_binary(
    name = "usb_forward_linux",
    srcs = [
        "usb_forward.cc",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        # Don't add anything else here. :usb_forward_windows still has to build it
        # without any other dependencies.
        "@libusb//:libusb_1_0",
    ],
)

genrule(
    name = "usb_forward_windows",
    srcs = [
        "usb_forward.cc",
        "@libusb_1_0_windows//file",
    ],
    outs = [
        "usb_forward.exe",
    ],
    cmd = " ".join([
        "$(location usb_forward_windows_build.sh)",
        "$(location usb_forward.cc)",
        "$(location @libusb_1_0_windows//file)",
        "$@",
    ]),
    output_to_bindir = True,
    tools = [
        "usb_forward_windows_build.sh",
        "@mingw_compiler//:all_files",
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
    visibility = ["//visibility:public"],
    deps = [
        "//motors:math",
        "//motors:motor",
        "//motors/peripheral:configuration",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
