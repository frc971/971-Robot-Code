licenses(["notice"])

cc_library(
    name = "jsont",
    srcs = [
        "jsont.c",
    ],
    hdrs = [
        "jsont.h",
    ],
    includes = ["."],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "jsont_test",
    srcs = ["test/test_tokenizer.c"],
    copts = [
        "-Wno-unused-parameter",
        "-Wno-unused-variable",
    ] + select({
        "@//tools:cpu_roborio": [
            "-Wno-unused-but-set-variable",
        ],
        "//conditions:default": [],
    }),
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":jsont"],
)
