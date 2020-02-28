load("@//tools/build_rules:select.bzl", "cpu_select")

cc_library(
    name = "webrtc",
    srcs = cpu_select({
        "arm": ["lib/arm/Release/libwebrtc_full.a"],
        "else": ["lib/x64/Release/libwebrtc_full.a"],
    }),
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
    ],
)
