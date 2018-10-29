package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stack_arena",
    srcs = ["stack_arena.cc"],
    hdrs = ["stack_arena.h"],
    deps = [
        "//aos/logging",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "stack_arena_test",
    srcs = ["stack_arena_test.cc"],
    deps = [
        ":stack_arena",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)
