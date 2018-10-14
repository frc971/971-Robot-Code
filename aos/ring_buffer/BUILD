package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ring_buffer",
    hdrs = [
        "ring_buffer.h",
    ],
)

cc_test(
    name = "ring_buffer_test",
    srcs = [
        "ring_buffer_test.cc",
    ],
    deps = [
        ":ring_buffer",
        "//aos/testing:googletest",
    ],
)
