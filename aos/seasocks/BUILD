py_binary(
    name = "gen_embedded",
    srcs = [
        "gen_embedded.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "seasocks_logger",
    srcs = ["seasocks_logger.cc"],
    hdrs = ["seasocks_logger.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/logging",
        "//third_party/seasocks",
    ],
)
