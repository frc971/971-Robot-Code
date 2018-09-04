load("//tools:environments.bzl", "mcu_cpus")

filegroup(
    name = "linkerscripts",
    srcs = [
        "kinetis_128k.ld",
        "kinetis_192k.ld",
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
        "time.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
)
