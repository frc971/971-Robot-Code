cc_library(
    name = "cccl",
    hdrs = glob(include = [
        "thrust/thrust/**",
        "libcudacxx/include/**",
        "cub/cub/**",
    ]),
    features = ["cuda"],
    includes = [
        "cub",
        "libcudacxx/include",
        "thrust",
    ],
    target_compatible_with = [
        "@//tools/platforms/gpu:nvidia",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
)
