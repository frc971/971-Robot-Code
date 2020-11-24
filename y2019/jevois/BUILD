load("//motors:macros.bzl", "hex_from_elf")

jevois_crc_args = [
    "$(location //third_party/pycrc:pycrc_main)",
    "--width=16",
    # This is the recommendation from
    # http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
    # for messages of 242 - 2048 bits, which covers what we want.
    # That's an analysis from an exhaustive search of all polynomials for
    # various CRCs to find the best ones. This is 0xBAAD, converted from the
    # weird format used there to the standard one used by pycrc.
    "--poly=0x755b",
    "--reflect-in=False",
    "--xor-in=0xffff",
    "--reflect-out=False",
    "--xor-out=0xffff",
    "--std=C99",
    "--algorithm=table-driven",
    "--symbol-prefix=jevois_crc_",
    "--crc-type=uint16_t",
]

genrule(
    name = "gen_jevois_crc",
    outs = [
        "jevois_crc.h",
        "jevois_crc.c",
    ],
    cmd = " && ".join([
        " ".join(jevois_crc_args + [
            "--generate=h",
            "--output=$(location jevois_crc.h)",
            "&&",
            "sed",
            "-i",
            "'s/\\(Generated on \\).*/\\1redacted/'",
            "$(location jevois_crc.h)",
        ]),
        " ".join(jevois_crc_args + [
            "--generate=c",
            "--output=$(location jevois_crc.c)",
            "&&",
            "sed",
            "-i",
            "'s/\\(Generated on \\).*/\\1redacted/'",
            "$(location jevois_crc.c)",
        ]),
    ]),
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//third_party/pycrc:pycrc_main",
    ],
)

cc_library(
    name = "jevois_crc",
    srcs = [
        "jevois_crc.c",
    ],
    hdrs = [
        "jevois_crc.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//third_party/GSL",
    ],
)

cc_library(
    name = "structures",
    hdrs = [
        "structures.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/containers:sized_array",
        "//aos/time",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "spi",
    srcs = [
        "spi.cc",
    ],
    hdrs = [
        "spi.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":jevois_crc",
        ":structures",
        "//aos/util:bitpacking",
        "//third_party/GSL",
    ] + select({
        "@platforms//os:linux": ["//aos/logging"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "uart",
    srcs = [
        "uart.cc",
    ],
    hdrs = [
        "uart.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":cobs",
        ":jevois_crc",
        ":structures",
        "//aos/containers:sized_array",
        "//aos/util:bitpacking",
        "//third_party/GSL",
    ] + select({
        "@platforms//os:linux": ["//aos/logging"],
        "//conditions:default": [],
    }),
)

cc_test(
    name = "uart_test",
    srcs = [
        "uart_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":uart",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "spi_test",
    srcs = [
        "spi_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":spi",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "cobs",
    hdrs = [
        "cobs.h",
    ],
    deps = [
        "//third_party/GSL",
    ],
)

cc_test(
    name = "cobs_test",
    srcs = [
        "cobs_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":cobs",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//third_party/GSL",
    ],
)

cc_library(
    name = "serial",
    srcs = ["serial.cc"],
    hdrs = ["serial.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/logging",
    ],
)

cc_binary(
    name = "teensy.elf",
    srcs = [
        "teensy.cc",
    ],
    target_compatible_with = ["//tools/platforms/hardware:cortex_m4f"],
    deps = [
        ":cobs",
        ":spi",
        ":uart",
        "//aos/time:time",
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:configuration",
        "//motors/peripheral:spi",
        "//motors/peripheral:uart",
        "//motors/print:usb",
        "//third_party/GSL",
        "//y2019/vision:constants",
    ],
)

hex_from_elf(
    name = "teensy",
    target_compatible_with = ["//tools/platforms/hardware:cortex_m4f"],
)
