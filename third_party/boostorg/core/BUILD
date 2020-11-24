licenses(["notice"])  # boost

cc_library(
    name = "core",
    hdrs = glob(["include/**"]),
    includes = ["include"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/boostorg/static_assert",
    ],
)
