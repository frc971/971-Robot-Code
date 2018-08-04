licenses(["notice"])  # boost

cc_library(
    name = "optional",
    hdrs = glob(["include/**"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/boostorg/assert",
        "//third_party/boostorg/core",
        "//third_party/boostorg/static_assert",
        "//third_party/boostorg/throw_exception",
        "//third_party/boostorg/type_traits",
    ],
)
