load("@//tools/build_rules:select.bzl", "compiler_select")

cc_library(
    name = "rawrtcdc",
    srcs = glob(
        [
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = [
            "src/sctp_redirect_transport",
            "src/crc32c/sse42*",
        ],
    ),
    hdrs = glob(["include/**/*.h"]),
    copts = compiler_select({
        "clang": [
            "-Wno-incompatible-pointer-types-discards-qualifiers",
        ],
        "gcc": [
            "-Wno-discarded-qualifiers",
            "-Wno-maybe-uninitialized",
        ],
    }) + [
        "-Wno-cast-qual",
        "-Wno-cast-align",
        "-Wno-missing-braces",
    ],
    defines = [
        "RAWRTCDC_HAVE_SCTP_REDIRECT_TRANSPORT=0",
        "RAWRTCDC_ENABLE_SSE42_CRC32C=0",
    ],
    includes = ["include/"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_rawrtc_rawrtc_common//:rawrtcc",
        "@com_github_rawrtc_usrsctp//:usrsctp_crc32",
    ],
)
