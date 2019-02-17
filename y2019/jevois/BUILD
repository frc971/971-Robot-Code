load("//tools:environments.bzl", "mcu_cpus")
load("//motors:macros.bzl", "hex_from_elf")

spi_crc_args = [
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
    "--symbol-prefix=jevois_spi_crc_",
    "--crc-type=uint16_t",
]

genrule(
    name = "gen_spi_crc",
    outs = [
        "spi_crc.h",
        "spi_crc.c",
    ],
    cmd = " && ".join([
        " ".join(spi_crc_args + [
            "--generate=h",
            "--output=$(location spi_crc.h)",
        ]),
        " ".join(spi_crc_args + [
            "--generate=c",
            "--output=$(location spi_crc.c)",
        ]),
    ]),
    tools = [
        "//third_party/pycrc:pycrc_main",
    ],
)

cc_library(
    name = "spi_crc",
    srcs = [
        "spi_crc.c",
    ],
    hdrs = [
        "spi_crc.h",
    ],
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
        "//third_party/eigen",
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
    deps = [
        ":spi_crc",
        ":structures",
        "//aos/util:bitpacking",
        "//third_party/GSL",
        "//third_party/optional",
    ],
)

cc_library(
    name = "uart",
    srcs = [
        "uart.cc",
    ],
    hdrs = [
        "uart.h",
    ],
    deps = [
        ":structures",
        "//aos/containers:sized_array",
        "//third_party/optional",
    ],
)

cc_test(
    name = "spi_test",
    srcs = [
        "spi_test.cc",
    ],
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
        "//aos/logging",
        "//third_party/GSL",
    ],
)

cc_test(
    name = "cobs_test",
    srcs = [
        "cobs_test.cc",
    ],
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
    deps = [
        "//aos/logging",
    ],
)

cc_binary(
    name = "teensy.elf",
    srcs = [
        "teensy.cc",
    ],
    restricted_to = ["//tools:cortex-m4f"],
    deps = [
        "//aos/time:time_mcu",
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:configuration",
        "//motors/peripheral:uart",
        "//motors/print:usb",
    ],
)

hex_from_elf(
    name = "teensy",
    restricted_to = ["//tools:cortex-m4f"],
)
