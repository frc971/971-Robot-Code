licenses(["notice"])  # boost

cc_library(
    name = "config",
    hdrs = glob(["include/**"]),
    includes = ["include"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/boostorg/core",
    ],
)
