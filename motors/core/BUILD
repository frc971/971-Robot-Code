load("//tools:environments.bzl", "mcu_cpus")

filegroup(
    name = "linkerscripts",
    srcs = [
        "kinetis_512_128.ld",
        "kinetis_512_256.ld",
        "kinetis_sections.ld",
    ],
    visibility = ["//tools/cpp:__pkg__"],
)

cc_library(
    name = "core",
    srcs = [
        "mk20dx128.c",
        "nonstd.c",
        "time.cc",
    ],
    hdrs = [
        "kinetis.h",
        "nonstd.h",
        "reg_debug.h",
        "time.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "semihosting",
    hdrs = [
        "semihosting.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/GSL",
    ],
)

cc_library(
    name = "itm",
    srcs = [
        "itm.cc",
    ],
    hdrs = [
        "itm.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        ":core",
    ],
)
