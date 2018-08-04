licenses(["notice"])  # boost

cc_library(
    name = "algorithm",
    hdrs = glob(["include/**"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/boostorg/assert",
        "//third_party/boostorg/static_assert",
        "//third_party/boostorg/type_traits",
    ],
)
