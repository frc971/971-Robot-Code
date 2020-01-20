load("@//tools/build_rules:select.bzl", "cpu_select")

cc_library(
    name = "webrtc",
    visibility = ["//visibility:public"],
    hdrs = glob(["include/**/*.h"]),
    srcs = cpu_select({
        "arm": ["lib/arm/Release/libwebrtc_full.a"],
        "else": ["lib/x64/Release/libwebrtc_full.a"],
    }),
    includes = ["include"],
    deps = [
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@com_google_absl//absl/algorithm:container",
    ],
)
