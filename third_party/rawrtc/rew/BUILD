load("@//tools/build_rules:select.bzl", "compiler_select")

cc_library(
    name = "rew",
    srcs = glob(
        [
            "src/**/*.c",
            "src/**/*.h",
        ],
    ),
    hdrs = glob(["include/*.h"]),
    copts = compiler_select({
        "clang": [
            "-Wno-incompatible-pointer-types-discards-qualifiers",
        ],
        "gcc": [
            "-Wno-discarded-qualifiers",
        ],
    }) + [
        "-Wno-cast-qual",
        "-Wno-incompatible-pointer-types-discards-qualifiers",
    ],
    includes = ["include/"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_rawrtc_re//:re"],
)
