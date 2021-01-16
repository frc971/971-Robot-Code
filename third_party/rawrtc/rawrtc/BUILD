load("@//tools/build_rules:select.bzl", "compiler_select")

cc_library(
    name = "rawrtc",
    srcs = glob([
        "src/**/*.c",
        "src/**/*.h",
    ]),
    hdrs = glob(["include/**/*.h"]),
    copts = compiler_select({
        "clang": [
            "-Wno-incompatible-pointer-types-discards-qualifiers",
        ],
        "gcc": [
            "-Wno-discarded-qualifiers",
            "-Wno-implicit-fallthrough",
        ],
    }) + [
        "-Wno-implicit-function-declaration",
        "-Wno-int-conversion",
        "-Wno-cast-qual",
        "-Wno-missing-braces",
        "-Iexternal/com_github_rawrtc_rawrtc/",
    ],
    includes = ["include/"],
    local_defines = [
        "RAWRTC_VERSION=\\\"0.5.1\\\"",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_rawrtc_rawrtc_common//:rawrtcc",
        "@com_github_rawrtc_rawrtc_data_channel//:rawrtcdc",
    ],
)
