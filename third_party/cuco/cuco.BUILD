cc_library(
    name = "cuco",
    hdrs = glob(include = ["include/**"]),
    defines = [
        "__CUDACC_RELAXED_CONSTEXPR__",
        "__CUDACC_EXTENDED_LAMBDA__",
    ],
    features = ["cuda"],
    includes = ["include"],
    target_compatible_with = [
        "@//tools/platforms/gpu:nvidia",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
)
