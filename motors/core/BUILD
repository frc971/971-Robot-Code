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
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "semihosting",
    hdrs = [
        "semihosting.h",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
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
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core",
    ],
)
