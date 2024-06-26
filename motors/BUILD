load("//motors:macros.bzl", "hex_from_elf")

cc_library(
    name = "motor",
    srcs = [
        "motor.cc",
    ],
    hdrs = [
        "motor.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        ":algorithms",
        ":util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/peripheral:configuration",
        "//motors/print",
    ],
)

cc_library(
    name = "util",
    hdrs = [
        "util.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
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
        "$(location @pandoc)",
        "-f",
        "markdown_github-hard_line_breaks",
        "-t",
        "html5",
        "-o",
        "$@",
        "$<",
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "@pandoc",
        "@pandoc//:all_files",
    ],
)

cc_library(
    name = "algorithms",
    srcs = [
        "algorithms.cc",
    ],
    hdrs = [
        "algorithms.h",
    ],
)

cc_test(
    name = "algorithms_test",
    srcs = [
        "algorithms_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
    visibility = ["//visibility:public"],
)

cc_test(
    name = "math_test",
    srcs = [
        "math_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":math",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "button_board.elf",
    srcs = [
        "button_board.cc",
    ],
    target_compatible_with = ["@platforms//os:none"],
    deps = [
        ":util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/print:usb",
        "//motors/usb",
        "//motors/usb:cdc",
        "//motors/usb:hid",
    ],
)

hex_from_elf(
    name = "button_board",
    target_compatible_with = ["@platforms//os:none"],
)

cc_binary(
    name = "simple_receiver.elf",
    srcs = [
        "simple_receiver.cc",
    ],
    copts = ["-Wno-type-limits"],
    target_compatible_with = ["@platforms//os:none"],
    deps = [
        ":util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/print:usb",
        "//motors/seems_reasonable:drivetrain_lib",
        "//motors/seems_reasonable:spring",
    ],
)

hex_from_elf(
    name = "simpler_receiver",
    target_compatible_with = ["@platforms//os:none"],
)

cc_binary(
    name = "simpler_receiver.elf",
    srcs = [
        "simpler_receiver.cc",
    ],
    copts = ["-Wno-type-limits"],
    target_compatible_with = ["@platforms//os:none"],
    deps = [
        ":util",
        "//motors/core",
        "//motors/peripheral:configuration",
        "//motors/print:usb",
        "//motors/seems_reasonable:drivetrain_lib",
    ],
)

hex_from_elf(
    name = "simple_receiver",
    target_compatible_with = ["@platforms//os:none"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
