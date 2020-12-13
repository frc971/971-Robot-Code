package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stack_arena",
    srcs = ["stack_arena.cc"],
    hdrs = ["stack_arena.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "stack_arena_test",
    srcs = ["stack_arena_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":stack_arena",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)
